#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <gradient_based_navigation/GradientBasedNavigationConfig.h>




bool GUI=false;  // show GUI
bool gbnEnabled=true;  // apply gbn (if false, cmd_vel goes through as it is)

int w=600;
int h=600;
cv::Mat imm=cv::Mat(h,w,CV_8U);
cv::Mat imm_attr=cv::Mat(h,w,CV_8U);
cv::Mat immTot=cv::Mat(h,w*2+2+2,CV_8U);
cv::Mat robot_grad,robot_grad_attr;
cv::Mat dist_ridotta;
cv::Mat visual_joy1(h/2,w/2,CV_8U);
cv::Mat visual_joy2(h/2,w/2,CV_8U);

double resolution=.05;
double max_vel_x=0.25;
double max_vel_theta=0.75;

// user-defined scan limits to filter out values outside range
double range_scan_min=0.25;
double range_scan_max=6;
double angle_scan_min=-M_PI/2+M_PI/8;
double angle_scan_max=M_PI/2-M_PI/8;

int size=0;
float angle_min=0;
float angle_incr=0;
std::vector<float> ranges;

float posx;
float posy;
int indx;
int indy;

float robot_posx=5; float robot_posy=5 ; float robot_orient=0;
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

bool obstacleNearnessEnabled = false; // if function to remain close to obstacles is enabled
float obstacleNearnessDistance = 0.5; // desired distance for obstacle nearness (in meters)

int momentum_scale_tb=150;
float momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);

float speed;

int sizematforze=round(grandezza_robot/resolution);
cv::Mat matriceForze,matriceForzeattr;

std::vector<geometry_msgs::Point32> attr_points;
float intensity=0;

bool laser_ready=false;
bool joystick_override_active = false;  // only joystick is considered

// Timestamps for measuring acquisition delays
ros::Time last_laser_msg_time;
ros::Time last_input_msg_time;
double MAX_MSG_DELAY=2.0; // (sec) Max delay from input or laser after which the robot is stopped.

float last_cmdvel_x, last_cmdvel_th; // last cmdvel messages sent

ros::Time last_movement = ros::Time(0); // last time robot moved
ros::Time last_stuck = ros::Time(0); // last time robot was stuck
bool stuck_recovery = false;
int stuck_trigger = 10; // seconds
int stuck_timeout = 5; // seconds

ros::NodeHandle *private_nh_ptr;



bool checkRobotStuck();



void parseCmdLine(int argc, char** argv){
    float sizer=20;
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

void get_max_vel() {
    private_nh_ptr->getParam("max_vel_x", max_vel_x);
    private_nh_ptr->getParam("max_vel_theta", max_vel_theta);
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

bool getobstacleNearnessEnabled(){
    bool par;
    private_nh_ptr->getParam("obstacleNearnessEnabled",par);
    return par;
}

float getobstacleNearnessDistance(){
    float par;
    private_nh_ptr->getParam("obstacleNearnessDistance",par);
    return par;
}

float getMinScanRange(){
    float par;
    private_nh_ptr->getParam("minScanRange",par);
    return par;
}

float getMaxScanRange(){
    float par;
    private_nh_ptr->getParam("maxScanRange",par);
    return par;
}

bool getgbnEnabled() {
    bool par;
    private_nh_ptr->getParam("gbnEnabled",par);
    return par;
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
        ROS_INFO("GradientBasedNavigation:: laser data ready!!!");
    }
    size=msg->ranges.size();
    angle_min=msg->angle_min;
    angle_incr=msg->angle_increment;
    time_stamp=msg->header.stamp;
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

    if (obstacleNearnessEnabled)
        desired_cmd_vel.angular.z=0;

    ros::Time tm = ros::Time::now();

    if (tm.sec - last_input_msg_time.sec > 2)
        last_movement = tm; // reset last_movement to restart counter after a while the robot was not moving

    if (checkRobotStuck()) {
        // printf("robot stuck!!! - recovery...\n");
        desired_cmd_vel.angular.z = 0;
        if (desired_cmd_vel.linear.x<0.1) desired_cmd_vel.linear.x=0.1;
    }


    last_input_msg_time = tm;
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

        double ang = angle_min+((float)(i)*angle_incr);

        if (ranges[i]>range_scan_min && ranges[i]<range_scan_max &&
            ang>angle_scan_min && ang<angle_scan_max ) {
            posx=ranges[i]*cos(ang);
            posy=ranges[i]*sin(ang);
            indy=-((posy/resolution)-h/2);
            indx=(-(posx/resolution)+w/2);

            newp.x=indy; newp.y=indx;
            if(oldp.x>=0&&oldp.y>=0&&oldp.x<w&&oldp.y<h){
                if((indx!=h/2||indy!=w/2)&&indx>=0&&indx<w && indy>=0&&indy<h&&(oldp.x!=h/2||oldp.y!=w/2)){
                    cv::line(imm,oldp,newp,cv::Scalar(0));
                    
                }
            }
            oldp=newp;
        } else {
            oldp.x=-1; oldp.y=-1;
        }
    }
}


/*** Function for building the stealth local view image's distance map ***/
void costruisciDistanceImageStealth(){
    cv::Mat imm2=imm(cv::Range(h/2-(4/resolution),h/2+(4/resolution)), cv::Range(w/2-(4/resolution),w/2+(4/resolution)));
    cv::distanceTransform( imm2, dist, distType, maskSize );
    //dist *= 1.f/n_pixel_sat; //1 su n pixel per la saturazione
    //cv::pow(dist, .5, dist);
    
    //set the parameters for the pick (translation d0 and decrease rate with respect to 0)
    float pheigth = 2;
    float w0 = 30;
    float d0 = obstacleNearnessDistance/resolution;  // LI was 10 
    //float d1 = 30;
    //set the parameters for the decrease rate with respect to +Inf
    float wInf = 50;

    for(int i=0;i<dist.rows;i+=1){
        for(int j=0;j<dist.cols;j+=1){
            //if(dist.at<float>(i,j)>1.f){
                //dist.at<float>(i,j)=1.f;
                
  
                float dist_elem = dist.at<float>(i,j);

                float base = dist_elem - d0;
                float exponent = - (pow(base,2) / w0);
                float exponentplusInf = - (base / wInf);
                //float exponentplusInf2 = - 0.2 * ((dist_elem - d1) / wInf);

                if(dist_elem > d0){
                    dist.at<float>(i,j) = pheigth * expf(exponentplusInf);
                }
               else{
                    dist.at<float>(i,j) = pheigth * expf(exponent);
                    //dist.at<float>(i,j) = pheigth * (1/ (1 + expf(exponentplusInf))) * (1/ (1 + expf(-exponentplusInf2)));
                }
            //}
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

/*** Function for building the standard (avoid obstacles) local view image's distance map ***/
void costruisciDistanceImageStandard(){
    cv::Mat imm2=imm(cv::Range(h/2-(4/resolution),h/2+(4/resolution)), cv::Range(w/2-(4/resolution),w/2+(4/resolution)));
    cv::distanceTransform( imm2, dist, distType, maskSize );
    dist *= 1.f/n_pixel_sat; //1 su n pixel per la saturazione
    cv::pow(dist, .5, dist);
    //printf("SONO DENTRO\n");
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
        if (joyspeed>max_vel_x){
            imm(cv::Range(0,imm.rows/2-10),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
            return;
        }
        imm(cv::Range(-(lungh/max_vel_x)*joyspeed+lungh,imm.rows/2-11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
    }else{
        if (joyspeed<-max_vel_x){
            imm(cv::Range(imm.rows/2+10,imm.rows),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
            return;
        }
        imm(cv::Range(imm.rows/2+11,-((imm.rows/2-11)/max_vel_x)*joyspeed+imm.rows/2+11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
    }
    if(joyangular>0){
        if (joyangular>max_vel_theta){
            imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(0,imm.cols/2-10))=cv::Scalar(150);
            return;
        }
        imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(-(((imm.cols)/2-10)/max_vel_theta)*joyangular+imm.cols/2-10,imm.cols/2-10))=cv::Scalar(150);
    }else{
        if (joyangular<-max_vel_theta){
            imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(imm.cols/2+10,imm.cols))=cv::Scalar(150);
            return;
        }
        imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(imm.cols/2+10,-((imm.cols/2-11)/max_vel_theta)*joyangular+imm.cols/2+11))=cv::Scalar(150);
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

void callbackReconfigure(gradient_based_navigation::GradientBasedNavigationConfig &config, uint32_t level){
    ROS_INFO("Reconfigure request");

    max_vel_x = config.max_vel_x;
    ROS_INFO("max_vel_x: %f",max_vel_x);

    max_vel_theta = config.max_vel_theta;
    ROS_INFO("max_vel_theta: %f", max_vel_theta);

    // distanza_saturazione_attr_cm = config.attractive_distance_influence;
    // distanza_saturazione_attr = distanza_saturazione_attr_cm/100.f;
    distanza_saturazione_attr = config.attractiveDistanceInfluence_m;
    n_pixel_sat_attr=(distanza_saturazione_attr)/resolution;
    private_nh_ptr->setParam("attractiveDistanceInfluence_m",distanza_saturazione_attr);
    ROS_INFO("attractive_distance_influence: %f", distanza_saturazione_attr);

    // distanza_saturazione_cm = config.obstacle_distance_influence;
    // distanza_saturazione = distanza_saturazione_cm/100.f;
    distanza_saturazione = config.obstaclesDistanceInfluence_m;
    n_pixel_sat=(distanza_saturazione)/resolution;
    private_nh_ptr->setParam("obstaclesDistanceInfluence_m",distanza_saturazione);
    ROS_INFO("obstacle_distance_influence: %f", distanza_saturazione);

    // force_scale_tb = config.force_scale;
    // force_scale=(force_scale_tb/1000.f)/(pixel_robot/2);
    force_scale=config.force_scale/(pixel_robot/2);
    force_scale_tb = (int)(config.force_scale * 1000);
    private_nh_ptr->setParam("force_scale",force_scale);
    ROS_INFO("force_scale (m): %f",force_scale*(pixel_robot/2));

    // momentum_scale_tb = config.momentum_scale;
    // momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);
    momentum_scale=config.momentum_scale/(pixel_robot/2);
    momentum_scale_tb = (int)(config.momentum_scale * 1000);
    private_nh_ptr->setParam("momentum_scale",(float)momentum_scale);
    ROS_INFO("momentum_scale (m): %f",momentum_scale*(pixel_robot/2));

    double robot_radius=config.robot_radius;
    private_nh_ptr->setParam("robot_radius",robot_radius);
    ROS_INFO("(NOT USED) robot_radius (m): %f",robot_radius);

}

// Last time a path has been seen
int last_path_time = 0; // sec of ros::Time 


void cbPath(const nav_msgs::Path::ConstPtr& msg) {
// Check if a path is being followed by a path planner

    std::vector<geometry_msgs::PoseStamped> v = msg->poses;
    ros::Time tm = msg->header.stamp;
    last_path_time = tm.sec; 
    // printf("  -- path size %lu at %d\n",v.size(), tm.sec);

}


// robot stuck if path to be reached and cmd_vel is zero
bool checkRobotStuck() {

  ros::Time tm = ros::Time::now();

  // first time set last_* values
  if (last_movement.sec==0) {
    last_movement = ros::Time::now();
    last_stuck = ros::Time::now();
  }

  if (stuck_recovery) { 
    if ((tm.sec-last_stuck.sec)<stuck_timeout)
      return true;
    else {
      stuck_recovery = false;
      last_movement = tm;
    }
  }

  //if (tm.sec-last_movement.sec>5)
  //  printf("  ++ [%2d] vel %.3f %.3f\n", tm.sec-last_movement.sec, last_cmdvel_x, last_cmdvel_th);

  // check cmd vel
  if ((fabs(last_cmdvel_x)>0.01) || (fabs(last_cmdvel_th)>0.4)) {
    last_movement = tm;
    return false;
  }

  // check path
  if (tm.sec - last_path_time>1) {
    return false;
  }

  if ((tm.sec-last_movement.sec)<stuck_trigger) {
    return false;
  }

  last_stuck = tm;
  stuck_recovery = true;
  return true;

}

const std::string scan_topic = "scan";

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gradient_based_navigation");

    parseCmdLine(argc, argv);

    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    private_nh_ptr = &private_nh;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    ros::Subscriber sub1 = n.subscribe(scan_topic, 1, callbackSensore);
    ros::Subscriber sub2 = n.subscribe("desired_cmd_vel", 1, callbackControllerInput);
    ros::Subscriber sub3 = n.subscribe("joystick_cmd_vel", 1, callbackJoystickInput);
    ros::Subscriber sub4 = n.subscribe("attractive_points", 1, callbackattractivePoints);
    ros::Subscriber sub5 = n.subscribe("path", 1, cbPath);

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
    if (!private_nh_ptr->hasParam("obstacleNearnessEnabled"))
      private_nh_ptr->setParam("obstacleNearnessEnabled",obstacleNearnessEnabled);
    if (!private_nh_ptr->hasParam("obstacleNearnessDistance"))
      private_nh_ptr->setParam("obstacleNearnessDistance",obstacleNearnessDistance);
    if (!private_nh_ptr->hasParam("gbnEnabled"))
      private_nh_ptr->setParam("gbnEnabled",gbnEnabled);

range_scan_min=getMinScanRange();
range_scan_max=getMaxScanRange();


    // Reconfigure settings

    dynamic_reconfigure::Server<gradient_based_navigation::GradientBasedNavigationConfig> server;
    dynamic_reconfigure::Server<gradient_based_navigation::GradientBasedNavigationConfig>::CallbackType f;

    f=boost::bind(&callbackReconfigure, _1, _2);
    server.setCallback(f);



    // ROS parameters

    // max vel parameters
    get_max_vel();   // also read in main loop

    // Update rate
    double fps=25.0; // Hz
    private_nh.param("rate", fps, 25.0);

    // GUI parameter
    private_nh.param("GUI", GUI, false);

    // ROS parameters - also read in main loop
    double par;
    private_nh_ptr->getParam("attractiveDistanceInfluence_m",par);
    distanza_saturazione_attr_cm=(int)(par*100);
    private_nh_ptr->getParam("obstaclesDistanceInfluence_m",par);
    distanza_saturazione_cm=(int)(par*100);
    private_nh_ptr->getParam("force_scale",par);
    force_scale_tb=(int)(par*1000);
    private_nh_ptr->getParam("momentum_scale",par);
    momentum_scale_tb=(int)(par*1000);
    obstacleNearnessEnabled=getobstacleNearnessEnabled();
    obstacleNearnessDistance=getobstacleNearnessDistance();
    range_scan_min=getMinScanRange();
    range_scan_max=getMaxScanRange();

    printf("gradient_based_navigation parameters\n");
    printf("  obstaclesDistanceInfluence_m: %.1f (m)\n",(double)distanza_saturazione_cm/100.0);
    printf("  force_scale: %.3f (m)\n",(double)force_scale_tb/1000);
    printf("  momentum_scale: %.3f (m)\n",(double)momentum_scale_tb/1000);
    printf("  max_vel_x: %.1f\n",max_vel_x);
    printf("  max_vel_theta: %.1f\n",max_vel_theta);
    printf("  rate: %.1f\n",fps);
    printf("  obstacleNearnessEnabled: %s\n",obstacleNearnessEnabled?"true":"false");
    printf("  minScanRange: %.2f\n",range_scan_min);
    printf("  maxScanRange: %.2f\n",range_scan_max);
    printf("  GUI: %s\n",(GUI?"true":"false"));
    printf("  enabled: %s\n",(gbnEnabled?"true":"false"));

    if (GUI) {
        // OpenCV stuff
        cv::namedWindow("GUI", 1);
        cv::createTrackbar("attractive distance influence (cm)", "GUI", &distanza_saturazione_attr_cm, 200, onTrackbarSaturazioneattrazione, &n);
        cv::createTrackbar("Obstacles distance influence (cm)", "GUI", &distanza_saturazione_cm, 200, onTrackbarSaturazione, &n);
        cv::createTrackbar("Force Scale (x1000)", "GUI", &force_scale_tb, 2000, onTrackbarForceScaling, 0);
        cv::createTrackbar("Momentum Scale (x1000)", "GUI", &momentum_scale_tb, 2000, onTrackbarMomentumScaling, 0);
    }
    ros::Rate loop_rate(fps);
    ros::AsyncSpinner spinner(4); // n threads
    spinner.start();

    float repulsive_linear_acc=0;
    float repulsive_angular_acc=0;
    float attr_linear_acc=0;
    float attr_angular_acc=0;
    cv::Vec3f forza;
    cv::Vec3f momento;

    ROS_INFO_STREAM("gradient_based_navigation: waiting for laser scan on topic " << scan_topic << " ...");
    while (!laser_ready && ros::ok()) {
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

    while (ros::ok()) { 

        /*** read ros params every fps iterations (1 second) *******************/
        if (iter==0){
            private_nh_ptr->getParam("GUI", GUI);
            get_max_vel();
            n_pixel_sat_attr=getNumPixelSaturazioneattrazione();
            n_pixel_sat=getNumPixelSaturazione();
            force_scale=getRepulsiveForceScale();
            momentum_scale=getRepulsiveMomentumScale();
            attr_dist_thresh=getattractiveDistanceThreshold();
            obstacleNearnessEnabled=getobstacleNearnessEnabled();
            obstacleNearnessDistance=getobstacleNearnessDistance();
            range_scan_min=getMinScanRange();
            range_scan_max=getMaxScanRange();
            gbnEnabled=getgbnEnabled();
        }
        iter++;
        if (iter>fps) iter=0;
        /************************************************************/


        /**** determine command_vel ****/
        if (!joystick_override_active && robot_was_moving() && delay_last_input()>MAX_MSG_DELAY) {
                ROS_INFO("No controller input detected, stopping the robot.");
                desired_cmd_vel.linear.x=0;  desired_cmd_vel.angular.z=0;    
                //joystick_override_active = true;
                //ros::param::set("use_only_joystick", 1);
        }
        if (delay_last_laser()>MAX_MSG_DELAY) {
            if (delay_last_laser()<4*MAX_MSG_DELAY){
                ROS_WARN("Stopping the robot: no laser!!!");
                joy_command_vel.linear.x=0;  joy_command_vel.angular.z=0;
            }            
        }
        if (!received_any_input || joystick_override_active)
            command_vel = joy_command_vel;
        else 
            command_vel = desired_cmd_vel;
    


        if (gbnEnabled) {

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
            if (obstacleNearnessEnabled) {
              costruisciDistanceImageStealth();
            }
            else {
              costruisciDistanceImageStandard();
            }
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

            target_linear_vel=command_vel.linear.x;
            target_ang_vel=command_vel.angular.z;
            speed=command_vel.linear.x;

            if(fabs(speed)>0.1){
                // std::cerr << "gbn:: speed " << speed << " " << fabs(speed)*1e3 << std::endl;
                calcolaMomentoeForza(matriceForze,momento,forza);
                repulsive_linear_acc=forza[1];
                repulsive_angular_acc=momento[2];

                //std::cerr << "gbn:: speed " << speed;
                //std::cerr << " target " << target_linear_vel << " " << target_ang_vel;
                //std::cerr << " repulsion " << repulsive_linear_acc << " " << repulsive_angular_acc;

                if(speed>0 && forza[1]>=0) {
                    target_linear_vel-=force_scale*repulsive_linear_acc*.01;
                    if (target_linear_vel<0) target_linear_vel=0;
                    if(fabs(target_linear_vel)<0.3 && fabs(target_ang_vel)>0.3) repulsive_angular_acc=0;
                    target_ang_vel+=momentum_scale*repulsive_angular_acc*.01;
                }
                else if(speed<0 && forza[1]<0) {
                    target_linear_vel-=force_scale*repulsive_linear_acc*.01; // LI ???
                    if (target_linear_vel<0) target_linear_vel=0; // LI ???
                    if(fabs(target_linear_vel)<0.3 && fabs(target_ang_vel)>0.3) repulsive_angular_acc=0;
                    target_ang_vel-=momentum_scale*repulsive_angular_acc*.01;
                }
                // else obstacle behind, so don't apply force

                //std::cerr << " -> " << repulsive_linear_acc << " " << repulsive_angular_acc;
                //std::cerr << " " << target_linear_vel << " " << target_ang_vel << std::endl;

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

            if(target_ang_vel>max_vel_theta){
                target_ang_vel=max_vel_theta;
            }
            if(target_ang_vel<-max_vel_theta){
                target_ang_vel=-max_vel_theta;
            }    
            if(target_linear_vel>max_vel_x){
                target_linear_vel=max_vel_x;
            }    
            if(target_linear_vel<-max_vel_x){
                target_linear_vel=-max_vel_x;
            }

            std::string esparam="0"; int iesparam=0;
            ros::param::get("emergency_stop", esparam);
            ros::param::get("emergency_stop", iesparam);

            if ((esparam=="1") || (iesparam==1)) {
              //std::cout << "Emergency Stop param: " << iesparam << std::endl;
              if (target_linear_vel < -0.3) target_linear_vel = -0.3; else target_linear_vel=0;
              target_ang_vel=0;
            }


            joystick_override_active = false;
            if (ros::param::get("use_only_joystick", esparam)) {
              if (esparam=="1") {
                  joystick_override_active = true;
              }
            }
            else if (ros::param::get("use_only_joystick", iesparam)) {
              if (iesparam==1) {
                  joystick_override_active = true;
              }
            }

            double max_linear_acc = 0.1;
            if (target_linear_vel > 0 && target_linear_vel - current_linear_vel > max_linear_acc){
                current_linear_vel = std::min((double)max_vel_x, (current_linear_vel+max_linear_acc));
                //std::cout << "target : " << target_linear_vel << " current : " << current_linear_vel << std::endl;
            }
            else if (target_linear_vel < 0 && target_linear_vel - current_linear_vel < -max_linear_acc){
                current_linear_vel = std::max(-(double)max_vel_x, (current_linear_vel-max_linear_acc));
                //std::cout << "target : " << target_linear_vel << " current : " << current_linear_vel << std::endl;
            }
            else
                current_linear_vel = target_linear_vel;

            current_ang_vel = target_ang_vel;

            command_vel.linear.x = current_linear_vel;
            command_vel.angular.z = current_ang_vel;

        }  // if (gbnEnabled)


        /**** publish result ****/        
        last_cmdvel_x = command_vel.linear.x;  // last cmdvel messages sent
        last_cmdvel_th = command_vel.angular.z;

        pub.publish(command_vel);
        /********************************************************************************/

        


        if (GUI) {
            ros::spinOnce();

            /**** Create the GUI ************************************************************/
            costruisciImmagineAssi(visual_joy1,target_linear_vel,target_ang_vel);
            costruisciImmagineAssi(visual_joy2,command_vel.linear.x,command_vel.angular.z);
            creaGUI(imm,dist_ridotta,robot_grad,visual_joy1,visual_joy2,immTot);

            cv::imshow("GUI",immTot);
            cv::waitKey(10);
            
            /********************************************************************************/
        }
        
        loop_rate.sleep();

    } // while ros.ok()


    spinner.stop();
    return 0;
}



