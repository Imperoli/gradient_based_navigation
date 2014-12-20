#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/Polygon.h>


bool GUI=false;

int w=600;
int h=600;
cv::Mat imm=cv::Mat(h,w,CV_8U);
cv::Mat imm_attr=cv::Mat(h,w,CV_8U);
cv::Mat immTot=cv::Mat(h,w*2+2+2,CV_8U);
cv::Mat robot_grad,robot_grad_attr;
cv::Mat dist_ridotta;
cv::Mat visual_joy1(h/2,w/2,CV_8U);
cv::Mat visual_joy2(h/2,w/2,CV_8U);

float resolution=.05;
float vel_angolare_max=.75;
float vel_lineare_max=.75;
float range_scan_min=0.001;
float range_scan_max=30;

int size=0;
float angle_min=0;
float angle_incr=0;
std::vector<float> ranges;

float posx;
float posy;
int indx;
int indy;

float robot_posx=0; float robot_posy=0 ; float robot_orient=0;
ros::Time time_stamp;

int maskSize = CV_DIST_MASK_PRECISE;
int distType = CV_DIST_L2;
cv::Mat dist;
cv::Mat dist_attr;

cv::Mat grad_x, grad_y;
cv::Mat grad;
int scale = 100;
int delta = 0;
int ddepth = CV_16S;

geometry_msgs::Twist joy_command_vel, desired_cmd_vel;
geometry_msgs::Twist command_vel;

int distanza_saturazione_cm= 65; int distanza_saturazione_attr_cm= 65;
float grandezza_robot=.8;// .35;
float pixel_robot=(grandezza_robot/resolution)*(grandezza_robot/resolution);
float distanza_saturazione=distanza_saturazione_cm/100.f; float distanza_saturazione_attr=distanza_saturazione_attr_cm/100.f;
float n_pixel_sat=(distanza_saturazione)/resolution; float n_pixel_sat_attr=(distanza_saturazione_attr)/resolution;
int force_scale_tb=500;
float force_scale=(force_scale_tb/1000.f)/(pixel_robot/2);
float attr_dist_thresh=.2;

int momentum_scale_tb=150;
float momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);

float speed;

int sizematforze=round(grandezza_robot/resolution);
cv::Mat matriceForze,matriceForzeattr;

std::vector<geometry_msgs::Point32> attr_points;
float intensity=0;

bool laser_ready=false;
bool joystick_override_active = true;

// Timestamps for measuring acquisition delays
ros::Time last_laser_msg_time;
ros::Time last_input_msg_time;
double MAX_MSG_DELAY=1.0; // (sec) Max delay from input or laser after which the robot is stopped.


ros::NodeHandle *private_nh_ptr;


void parseCmdLine(int argc, char** argv){
	float sizer=80;
	if (argc>1){
		sizer=atof(argv[1]);
		if (sizer>400||sizer<10){
			sizer=80;
		}
	}
	grandezza_robot=sizer/100.f;
	pixel_robot=(grandezza_robot/resolution)*(grandezza_robot/resolution);
	sizematforze=round(grandezza_robot/resolution);
	matriceForze=cv::Mat(sizematforze,sizematforze,CV_32FC3);
}

float getattractiveDistanceThreshold(){
	double dist_thresh;
	private_nh_ptr->getParam("attractiveDistanceThreshold_m",dist_thresh);
	//ros::param::get("attractiveDistanceThreshold_m",dist_thresh);
	return dist_thresh;
}

float getNumPixelSaturazioneattrazione(){
	double dist_sat;
	private_nh_ptr->getParam("attractiveDistanceInfluence_m",dist_sat);
	//ros::param::get("attractiveDistanceInfluence_m",dist_sat);
	return dist_sat/resolution;
}

float getNumPixelSaturazione(){
	double dist_sat;
	private_nh_ptr->getParam("obstaclesDistanceInfluence_m",dist_sat);
	//ros::param::get("obstaclesDistanceInfluence_m",dist_sat);
	return dist_sat/resolution;
}

float getRepulsiveForceScale(){
	double scale;
	private_nh_ptr->getParam("force_scale",scale);
	//ros::param::get("force_scale",scale);
	return scale/(pixel_robot/2);
}

float getRepulsiveMomentumScale(){
	double scale;
	private_nh_ptr->getParam("momentum_scale",scale);
	//ros::param::get("momentum_scale",scale);
	return scale/(pixel_robot/2);
}

/*** Callback for retrieving attractive Points ***/
void callbackattractivePoints(const geometry_msgs::Polygon::ConstPtr& msg)
{	
	attr_points.clear();
	attr_points=msg->points;
}

bool stoppedForCloseObject = false;
ros::Time last_close_object_time;
const double CLOSE_DETECTION_RESET_TIME = 3.0;
// Check very close obstacles (<0.2 m) and stop the robot
void very_close_obstacle_check() {
    int cnt=0, dim=60, step=2;
    for (int i=size/2-(dim/2)*step; i<size/2+(dim/2)*step; i+=step)
      if (ranges[i]<0.2)
		cnt++;
    //std::cout << "Closeness " << cnt << std::endl;
    double very_close_obstacle = (double)cnt/dim;
    if (very_close_obstacle>0.5) {
		ROS_WARN("Very close obstacle: stopping the robot");
		last_close_object_time = ros::Time::now();
		ros::param::set("emergency_stop", 1);
		stoppedForCloseObject = true;
    }
    else if(stoppedForCloseObject){
		double delta_time = (ros::Time::now()-last_close_object_time).toSec();
		if(delta_time>CLOSE_DETECTION_RESET_TIME){
			ROS_INFO("%g seconds have passed without finding close obstacles, will re-enable movement",CLOSE_DETECTION_RESET_TIME);
			stoppedForCloseObject=false;
			ros::param::set("emergency_stop", 0);
		}
    }
}


/*** Callback for retrieving the laser scan ***/
void callbackSensore(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
  if (!laser_ready) {
    laser_ready=true;
    std::cout << "GradientBasedNavigation:: laser data ready!!!" << std::endl;
  }
	size=msg->ranges.size();
	angle_min=msg->angle_min;
	angle_incr=msg->angle_increment;
	time_stamp=msg->header.stamp;
	range_scan_min=msg->range_min;
	range_scan_max=msg->range_max;
	ranges=msg->ranges;
	last_laser_msg_time = time_stamp; //ros::Time::now();
	very_close_obstacle_check();
}


/*** Callback for retrieving the joystick data ***/
void callbackJoystickInput(const geometry_msgs::Twist::ConstPtr& msg)
{	
	joy_command_vel.linear=msg->linear;
	joy_command_vel.angular=msg->angular;
	//last_input_msg_time = ros::Time::now();
	//std::cout << "Received joystick " << joy_command_vel.linear.x << std::endl;
}

bool received_any_input = false;
/*** Callback for retrieving the high level controller output***/
void callbackControllerInput(const geometry_msgs::Twist::ConstPtr& msg)
{	
	desired_cmd_vel.linear=msg->linear;
	desired_cmd_vel.angular=msg->angular;
	last_input_msg_time = ros::Time::now();
	if(!received_any_input){ //this is verified only the first time it receives a message
		ROS_INFO("Received first controller input - joystick is temporaly disabled");
		received_any_input = true;
	}
}

bool robot_was_moving(){
	return !((desired_cmd_vel.linear.x==0) && (desired_cmd_vel.angular.z == 0));
}

double delay_last_input() {
    ros::Time current_time = ros::Time::now();
    double delta_time = (current_time-last_input_msg_time).toSec();
    return delta_time;
}

double delay_last_laser() {
    ros::Time current_time = ros::Time::now();
    double delta_time = (current_time-last_laser_msg_time).toSec();
    return delta_time;
}


void costruisciattractiveField(){
	float posx=0; float posy=0; int indx=0; int indy=0; float rx=0; float ry=0;
	intensity=0;
	cv::Mat imm2,dist_temp,matriceForzeattr_temp,robot_grad_attr_temp;
	dist_attr=cv::Mat::zeros(8/resolution,8/resolution,CV_32FC1);
	matriceForzeattr=cv::Mat::zeros(sizematforze,sizematforze,CV_32FC3);
	robot_grad_attr=cv::Mat::zeros(h/2,w/2,CV_8UC1);

	for(int i = 0; i < attr_points.size(); i++){
		imm_attr = cv::Scalar(255);
		matriceForzeattr_temp=cv::Mat::zeros(sizematforze,sizematforze,CV_32FC3);
		robot_grad_attr_temp=cv::Mat::zeros(robot_grad_attr.size(),robot_grad_attr.type());		
		//posx=attr_points[i].x;
		//posy=attr_points[i].y;
		posx=-attr_points[i].y*sin(robot_orient)+attr_points[i].x*cos(robot_orient)+robot_posx;
		posy=(attr_points[i].x*sin(robot_orient)+attr_points[i].y*cos(robot_orient))+robot_posy;
		indy=-((posy/resolution)-h/2);
		indx=(-(posx/resolution)+w/2);

		if(indy>=0&&indy<h&&indx>=0&&indx<w){
			if(GUI){
				int size2=5;
				for (int k=-size2/2;k<size2/2;k++){
					for (int k2=-size2/2;k2<size2/2;k2++){
						imm.at<uchar>(indx+k,indy+k2)=50;
					}
				}
			}	
			imm_attr.at<uchar>(indx,indy)=0;
			imm2=imm_attr(cv::Range(h/2-(4/resolution),h/2+(4/resolution)), cv::Range(w/2-(4/resolution),w/2+(4/resolution)));
			cv::distanceTransform( imm2, dist_temp, distType, maskSize );
			intensity=attr_points[i].z;

			dist_temp *= 1.f/(n_pixel_sat_attr); //1 su n pixel per la saturazione
			//cv::pow(dist_temp, .5, dist_temp);
			for(int i=0;i<dist_temp.rows;i+=1){
				for(int j=0;j<dist_temp.cols;j+=1){
					if(dist_temp.at<float>(i,j)>1){
						dist_temp.at<float>(i,j)=1;
					}
				}
			}

			cv::Mat abs_grad_x, abs_grad_y;
			cv::Scharr( dist_temp, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );
			cv::convertScaleAbs( grad_x, abs_grad_x );
			cv::Scharr( dist_temp, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );
			cv::convertScaleAbs( grad_y, abs_grad_y );
			cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

			cv::Mat grad_x_rid=grad_x(cv::Range(grad_x.rows/2-(sizematforze)/2,grad_x.rows/2+(sizematforze)/2), cv::Range(grad_x.cols/2-(sizematforze)/2,grad_x.cols/2+(sizematforze)/2));
			cv::Mat grad_y_rid=grad_y(cv::Range(grad_y.rows/2-(sizematforze)/2,grad_y.rows/2+(sizematforze)/2), cv::Range(grad_y.cols/2-(sizematforze)/2,grad_y.cols/2+(sizematforze)/2));
	
			/// costruzione matrice forze
			for(int i=0;i<sizematforze;i++){
				for(int j=0;j<sizematforze;j++){
					matriceForzeattr_temp.at<cv::Vec3f>(i,j)=cv::Vec3f(grad_x_rid.at<short>(i,j),grad_y_rid.at<short>(i,j),0)*intensity;
				}
			}
			cv::Mat gradrid=grad(cv::Range(grad.rows/2-(sizematforze)/2,grad.rows/2+(sizematforze)/2), cv::Range(grad.cols/2-(sizematforze)/2,grad.cols/2+(sizematforze)/2));	
			cv::resize(gradrid, robot_grad_attr_temp, cv::Size(h/2,w/2), 0, 0,cv::INTER_LINEAR);

			robot_grad_attr+=robot_grad_attr_temp;
			matriceForzeattr+=matriceForzeattr_temp;
			dist_attr=dist_attr+dist_temp;

			float dist_=sqrt(posx*posx+posy*posy);
			if(dist_<=attr_dist_thresh){
				//attr_points.erase(attr_points.begin()+i);
				//i--;
				robot_grad_attr-=robot_grad_attr_temp;
				matriceForzeattr-=matriceForzeattr_temp;
				dist_attr=dist_attr-dist_temp;		
			}
		}
	}
}

/*** Function for building the local view image ***/
void costruisciScanImage(){
	imm = cv::Scalar(255);
	posx=0; posy=0; indx=0; indy=0;
	cv::Point oldp(-1,-1);
	cv::Point newp;
	
	for(int i = 0; i < size; i++){
		
		if(ranges[i]>range_scan_min&&ranges[i]<range_scan_max){
			posx=ranges[i]*cos(angle_min+((float)(i)*angle_incr));
			posy=ranges[i]*sin(angle_min+((float)(i)*angle_incr));
			indy=-((posy/resolution)-h/2);
			indx=(-(posx/resolution)+w/2);

			newp.x=indy; newp.y=indx;
			if(oldp.x>=0&&oldp.y>=0&&oldp.x<w&&oldp.y<h){
				if((indx!=h/2||indy!=w/2)&&indx>=0&&indx<w && indy>=0&&indy<h&&(oldp.x!=h/2||oldp.y!=w/2)){
					cv::line(imm,oldp,newp,cv::Scalar(0));
					
				}
			}
			oldp=newp;
		}else{
			oldp.x=-1; oldp.y=-1;
		}
	}
}


/*** Function for building the local view image's distance map ***/
void costruisciDistanceImage(){
	cv::Mat imm2=imm(cv::Range(h/2-(4/resolution),h/2+(4/resolution)), cv::Range(w/2-(4/resolution),w/2+(4/resolution)));
	cv::distanceTransform( imm2, dist, distType, maskSize );
	dist *= 1.f/n_pixel_sat; //1 su n pixel per la saturazione
	cv::pow(dist, .5, dist);
	for(int i=0;i<dist.rows;i+=1){
		for(int j=0;j<dist.cols;j+=1){
			if(dist.at<float>(i,j)>1.f){
				dist.at<float>(i,j)=1.f;
			}
		}
	}
	if(GUI){	
		int size2=round(grandezza_robot/resolution);
		for (int i=-size2/2;i<size2/2;i++){
			for (int j=-size2/2;j<size2/2;j++){
				imm.at<uchar>(h/2+i,w/2+j)=100;
			}
		}
	}
}


/*** Function for building gradient map ***/
void costruisciGradientImage(){

	/// Generate grad_x and grad_y
	cv::Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	cv::Scharr( dist, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );
	//Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	cv::convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	cv::Scharr( dist, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );
	//Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	cv::convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	
	cv::Mat grad_x_rid=grad_x(cv::Range(grad_x.rows/2-(sizematforze)/2,grad_x.rows/2+(sizematforze)/2), cv::Range(grad_x.cols/2-(sizematforze)/2,grad_x.cols/2+(sizematforze)/2));
	cv::Mat grad_y_rid=grad_y(cv::Range(grad_y.rows/2-(sizematforze)/2,grad_y.rows/2+(sizematforze)/2), cv::Range(grad_y.cols/2-(sizematforze)/2,grad_y.cols/2+(sizematforze)/2));
	
	/// costruzione matrice forze
	for(int i=0;i<sizematforze;i++){
		for(int j=0;j<sizematforze;j++){
			matriceForze.at<cv::Vec3f>(i,j)=cv::Vec3f(grad_x_rid.at<short>(i,j),grad_y_rid.at<short>(i,j),0);
		}
	}
	cv::Mat gradrid=grad(cv::Range(grad.rows/2-(sizematforze)/2,grad.rows/2+(sizematforze)/2), cv::Range(grad.cols/2-(sizematforze)/2,grad.cols/2+(sizematforze)/2));	
	cv::resize(gradrid, robot_grad, cv::Size(h/2,w/2), 0, 0,cv::INTER_LINEAR);
	cv::resize(dist, dist_ridotta, cv::Size(h/2,w/2), 0, 0,cv::INTER_LINEAR);
	dist_ridotta*=255;
	dist_ridotta.convertTo(dist_ridotta,CV_8UC1);
}


/*** Function used for the visualization of the command velocity in the GUI ***/
void costruisciImmagineAssi(cv::Mat& imm, float joyspeed, float joyangular){
	imm=cv::Scalar(255);
	int lungh=imm.rows/2-10;
	
	imm(cv::Range(0,imm.rows/2-10),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2+10,imm.rows),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(0,imm.cols/2-10))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(imm.cols/2+10,imm.cols))=cv::Scalar(0);

	if(joyspeed>0){
		if (joyspeed>vel_lineare_max){
			imm(cv::Range(0,imm.rows/2-10),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(-(lungh/vel_lineare_max)*joyspeed+lungh,imm.rows/2-11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
	}else{
		if (joyspeed<-vel_lineare_max){
			imm(cv::Range(imm.rows/2+10,imm.rows),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2+11,-((imm.rows/2-11)/vel_lineare_max)*joyspeed+imm.rows/2+11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
	}
	if(joyangular>0){
		if (joyangular>vel_angolare_max){
			imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(0,imm.cols/2-10))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(-(((imm.cols)/2-10)/vel_angolare_max)*joyangular+imm.cols/2-10,imm.cols/2-10))=cv::Scalar(150);
	}else{
		if (joyangular<-vel_angolare_max){
			imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(imm.cols/2+10,imm.cols))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(imm.cols/2+10,-((imm.cols/2-11)/vel_angolare_max)*joyangular+imm.cols/2+11))=cv::Scalar(150);
	}
}


void onTrackbarSaturazioneattrazione( int,void*){
	distanza_saturazione_attr=distanza_saturazione_attr_cm/100.f;
	n_pixel_sat_attr=(distanza_saturazione_attr)/resolution;
	private_nh_ptr->setParam("attractiveDistanceInfluence_m",distanza_saturazione_attr);
	//ros::param::set("attractiveDistanceInfluence_m",distanza_saturazione_attr);	
}

void onTrackbarSaturazione( int,void*){
	distanza_saturazione=distanza_saturazione_cm/100.f;
	n_pixel_sat=(distanza_saturazione)/resolution;
	private_nh_ptr->setParam("obstaclesDistanceInfluence_m",distanza_saturazione);	
	//ros::param::set("obstaclesDistanceInfluence_m",distanza_saturazione);	
}

void onTrackbarForceScaling( int, void* ){
	force_scale=(force_scale_tb/1000.f)/(pixel_robot/2);
	private_nh_ptr->setParam("force_scale",(float)force_scale_tb/1000.f);
	//ros::param::set("force_scale",(float)force_scale_tb/1000.f);
}

void onTrackbarMomentumScaling( int, void* ){
	momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);
	private_nh_ptr->setParam("momentum_scale",(float)momentum_scale_tb/1000.f);
	//ros::param::set("momentum_scale",(float)momentum_scale_tb/1000.f);
}


/*** Building the GUI ***/
void creaGUI(cv::Mat& imm1, cv::Mat& imm2, cv::Mat& imm3, cv::Mat& imm4, cv::Mat& imm5, cv::Mat& immris){
	immris=cv::Scalar(200);
	for (int i=0;i<imm1.rows;i++){
		for(int j=0;j<imm1.cols;j++){
			immris.at<uchar>(i,j)=imm1.at<uchar>(i,j);
		}
	}
	for (int i=0;i<imm2.rows-1;i++){
		for(int j=0;j<imm2.cols;j++){
			immris.at<uchar>(i,j+imm1.cols+2)=imm2.at<uchar>(i,j);
		}
	}
	for (int i=1;i<imm3.rows;i++){
		for(int j=0;j<imm3.cols;j++){
			immris.at<uchar>(i+imm2.rows,j+imm1.cols+2)=imm3.at<uchar>(i,j);
		}
	}
	for (int i=0;i<imm4.rows-1;i++){
		for(int j=0;j<imm4.cols;j++){
			immris.at<uchar>(i,j+imm1.cols+2+imm2.cols+2)=imm4.at<uchar>(i,j);
		}
	}
	for (int i=1;i<imm5.rows;i++){
		for(int j=0;j<imm5.cols;j++){
			immris.at<uchar>(i+imm4.rows,j+imm1.cols+2+imm2.cols+2)=imm5.at<uchar>(i,j);
		}
	}
}


void calcolaMomentoeForza(cv::Mat& forze, cv::Vec3f& momento, cv::Vec3f& forza){
	cv::Vec3f momtemp(0,0,0);
	cv::Vec3f forzatemp(0,0,0);
	cv::Vec3f f(0,0,0);
	cv::Vec3f b(0,0,0);
	for (int i=0;i<forze.cols;i++){
		for (int j=0; j<forze.rows;j++){
			f=forze.at<cv::Vec3f>(j,i);
			if(f!=cv::Vec3f(0,0,0)&&(i!=forze.rows/2 && j!=forze.rows/2)){
				if(j<=forze.rows/2){
					b[0]=-(-i+forze.rows/2); b[1]=-(j-forze.rows/2);
					momtemp+=b.cross(f);
				}
			}
			if(f!=cv::Vec3f(0,0,0)){
				forzatemp[0]+=f[0]; forzatemp[1]+=f[1];
			}
		}
	}
	momento=momtemp;
	forza=forzatemp;
} 


int main(int argc, char **argv)
{

  	ros::init(argc, argv, "gradient_based_navigation");

	parseCmdLine(argc, argv);

  	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	private_nh_ptr = &private_nh;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	ros::Subscriber sub3 = n.subscribe("joystick_cmd_vel", 1, callbackJoystickInput);
	ros::Subscriber sub2 = n.subscribe("desired_cmd_vel", 1, callbackControllerInput);

	ros::Subscriber sub = n.subscribe("base_scan", 1, callbackSensore);

	ros::Subscriber sub4 = n.subscribe("attractive_points", 1, callbackattractivePoints);

	if (!private_nh_ptr->hasParam("attractiveDistanceThreshold_m"))
	  private_nh_ptr->setParam("attractiveDistanceThreshold_m",attr_dist_thresh);
	if (!private_nh_ptr->hasParam("attractiveDistanceInfluence_m"))
	  private_nh_ptr->setParam("attractiveDistanceInfluence_m",0.5f);
	if (!private_nh_ptr->hasParam("obstaclesDistanceInfluence_m"))
	  private_nh_ptr->setParam("obstaclesDistanceInfluence_m",1.0f);
	if (!private_nh_ptr->hasParam("force_scale"))
	  private_nh_ptr->setParam("force_scale",.4f);
	if (!private_nh_ptr->hasParam("momentum_scale"))
	  private_nh_ptr->setParam("momentum_scale",.1f);


	// GUI parameter
	private_nh.param("GUI", GUI, false);

	// parametri ROS - letti nel ciclo
	double par;
	private_nh_ptr->getParam("attractiveDistanceInfluence_m",par);
	distanza_saturazione_attr_cm=(int)(par*100);
	private_nh_ptr->getParam("obstaclesDistanceInfluence_m",par);
	distanza_saturazione_cm=(int)(par*100);
	private_nh_ptr->getParam("force_scale",par);
	force_scale_tb=(int)(par*1000);
	private_nh_ptr->getParam("momentum_scale",par);
	momentum_scale_tb=(int)(par*1000);
	
	// OpenCV stuff
	cv::namedWindow("GUI", 1);
	cv::createTrackbar("attractive distance influence (cm)", "GUI", &distanza_saturazione_attr_cm, 200, onTrackbarSaturazioneattrazione, &n);
	cv::createTrackbar("Obstacles distance influence (cm)", "GUI", &distanza_saturazione_cm, 200, onTrackbarSaturazione, &n);
	cv::createTrackbar("Force Scale", "GUI", &force_scale_tb, 2000, onTrackbarForceScaling, 0);
	cv::createTrackbar("Momentum Scale", "GUI", &momentum_scale_tb, 2000, onTrackbarMomentumScaling, 0);

    // parametro ROS
	int fps=100;
	ros::Rate loop_rate(fps);
	ros::AsyncSpinner spinner(8); // n threads
	spinner.start();

	float repulsive_linear_acc=0;
	float repulsive_angular_acc=0;
	float attr_linear_acc=0;
	float attr_angular_acc=0;
	cv::Vec3f forza;
	cv::Vec3f momento;

	ROS_INFO("gradient_based_navigation: waiting for laser scan...");
	while (!laser_ready) {
	  loop_rate.sleep();
	}
	ROS_INFO("gradient_based_navigation: laser scan OK");
	
	
  double current_linear_vel=0;
  double target_linear_vel=0;
  double current_ang_vel=0;
  double target_ang_vel=0;

	tf::Vector3 axis;
	tf::TransformListener listener;
	std::string laser_frame;
	std::string map_frame;
	tf::StampedTransform transform;

	private_nh_ptr->param("laser_frame",laser_frame,std::string("base_laser_link"));
	private_nh_ptr->param("map_frame",map_frame,std::string("map"));

	if (n.hasParam("tf_prefix")) {
	    std::string tf_prefix;
	    n.getParam("tf_prefix",tf_prefix);
	    ROS_INFO_STREAM("Using tf_prefix: " << tf_prefix);
	    laser_frame = "/" + tf_prefix + "/" + laser_frame;
	    map_frame = "/" + tf_prefix + "/" + map_frame;
	}
	    
	ros::Duration delay(3.0); // seconds
	delay.sleep();
	
	int iter=0;

	while(n.ok()){
		/*** read ros params every 100 iterations *******************/
		if (iter==0){
			private_nh_ptr->getParam("GUI", GUI);
			n_pixel_sat_attr=getNumPixelSaturazioneattrazione();
			n_pixel_sat=getNumPixelSaturazione();
			force_scale=getRepulsiveForceScale();
			momentum_scale=getRepulsiveMomentumScale();
			attr_dist_thresh=getattractiveDistanceThreshold();
		}
		iter++;
		if(iter>fps) iter=0;
		/************************************************************/
	
		if (attr_points.size()>0) {
		  /*** tf *************************/
		  try{
			  listener.lookupTransform(laser_frame, map_frame, time_stamp, transform);
			  }
		  catch (tf::TransformException ex){
			ROS_ERROR("gradient_based_navigation: %s",ex.what());
			ROS_ERROR_STREAM("TF from " << laser_frame << "  to " << map_frame);
			//std::cout<<source_frame<<std::endl;
		  }

		  robot_posx=transform.getOrigin().x();
		  robot_posy=transform.getOrigin().y();
		  robot_orient=transform.getRotation().getAngle();
		  axis=transform.getRotation().getAxis();		
		  robot_orient*=axis[2];
		  /********************************/
		}
		
		/*** Building the force field ***/
		costruisciScanImage();
		costruisciDistanceImage();
		costruisciGradientImage();
		////
		costruisciattractiveField();
		////
		robot_grad+=robot_grad_attr;
		/********************************/

		/*** Compute the velocity command and publish it ********************************/
		repulsive_linear_acc=0;
		repulsive_angular_acc=0;
		
		
		// Check if input or laser messages are too old
#if 0
		// Does not work with joystick
		if (delay_last_input()>MAX_MSG_DELAY) {
		    if (delay_last_input()<4*MAX_MSG_DELAY)
			ROS_WARN("Stopping the roobt: no input !!!");
		    joy_command_vel.linear.x=0;  joy_command_vel.angular.z=0;
		}
#endif	
		if (!joystick_override_active && robot_was_moving() && delay_last_input()>MAX_MSG_DELAY) {
				ROS_INFO("No controller input detected, switching to Joystick only control");
				desired_cmd_vel.linear.x=0;  desired_cmd_vel.angular.z=0;	
				joystick_override_active = true;
				ros::param::set("use_only_joystick", 1);
		}
		if (delay_last_laser()>MAX_MSG_DELAY) {
		    if (delay_last_laser()<4*MAX_MSG_DELAY){
				ROS_WARN("Stopping the robot: no laser!!!");
				joy_command_vel.linear.x=0;  joy_command_vel.angular.z=0;
		    }
			
		}
		if(!received_any_input || joystick_override_active)
			command_vel=joy_command_vel;
		else command_vel = desired_cmd_vel;

		target_linear_vel=command_vel.linear.x;
		target_ang_vel=command_vel.angular.z;
		speed=command_vel.linear.x;

		if(speed!=0){
			calcolaMomentoeForza(matriceForze,momento,forza);
			repulsive_linear_acc=forza[1];		
			repulsive_angular_acc=momento[2];		
			if(speed>0&&forza[1]>0){	
				target_linear_vel-=force_scale*repulsive_linear_acc*.01;
				target_ang_vel+=momentum_scale*repulsive_angular_acc*.01;
			}else if(speed<0&&forza[1]<0){
				target_linear_vel-=force_scale*repulsive_linear_acc*.01;
				target_ang_vel-=momentum_scale*repulsive_angular_acc*.01;
			}
		}	

		if (target_linear_vel*speed<0){
			target_linear_vel=0;
		}

		///////////// attrazione ///////////////////////
		if(speed>0){
			calcolaMomentoeForza(matriceForzeattr,momento,forza);
			attr_linear_acc=forza[1];		
			attr_angular_acc=momento[2];		
			if(forza[1]>0){	
				target_linear_vel+=force_scale*attr_linear_acc*.01;
				target_ang_vel-=momentum_scale*attr_angular_acc*.01;
			}
		}	
		////////////////////////////////////////////////

		if(target_ang_vel>vel_angolare_max){
			target_ang_vel=vel_angolare_max;
		}
		if(target_ang_vel<-vel_angolare_max){
			target_ang_vel=-vel_angolare_max;
		}	
		if(target_linear_vel>vel_lineare_max){
			target_linear_vel=vel_lineare_max;
		}	
		if(target_linear_vel<-vel_lineare_max){
			target_linear_vel=-vel_lineare_max;
		}
    std::string esparam; int iesparam;
    if (ros::param::get("emergency_stop", esparam))
    {
      if (esparam=="1") {
          target_linear_vel=0; target_ang_vel=0;
          //std::cout << "Emergency Stop param: " << esparam << std::endl;
      }
    }
    else if (ros::param::get("emergency_stop", iesparam))
    {
      if (iesparam==1) {
          target_linear_vel=0; target_ang_vel=0;
          //std::cout << "Emergency Stop param: " << iesparam << std::endl;
      }
    }
    joystick_override_active = false;
    if (ros::param::get("use_only_joystick", esparam))
    {
      if (esparam=="1") {
          joystick_override_active = true;
      }
    }
    else if (ros::param::get("use_only_joystick", iesparam))
    {
      if (iesparam==1) {
          joystick_override_active = true;
      }
    }

    double max_linear_acc = 0.05;
    if (target_linear_vel > 0 && target_linear_vel - current_linear_vel > max_linear_acc){
		current_linear_vel = std::min((double)vel_lineare_max, (current_linear_vel+max_linear_acc));
		//std::cout << "target : " << target_linear_vel << " current : " << current_linear_vel << std::endl;
	}
	else
    	current_linear_vel = target_linear_vel;

    current_ang_vel = target_ang_vel;

   command_vel.linear.x = current_linear_vel;
	command_vel.angular.z = current_ang_vel;
	pub.publish(command_vel);
	/********************************************************************************/

		
		if(GUI){
			/**** Create the GUI ************************************************************/
			costruisciImmagineAssi(visual_joy1,target_linear_vel,target_ang_vel);
			costruisciImmagineAssi(visual_joy2,command_vel.linear.x,command_vel.angular.z);
			creaGUI(imm,dist_ridotta,robot_grad,visual_joy1,visual_joy2,immTot);

			cv::imshow("GUI",immTot);
			cv::waitKey(1000/fps);
			/********************************************************************************/
		}


		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}



