#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>

ros::Publisher pub;

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "send_attractive_points");


  	ros::NodeHandle n;

	pub = n.advertise<geometry_msgs::Polygon>("attractive_points", 1);

	int fps=100;
	ros::Rate loop_rate(fps);
	ros::AsyncSpinner spinner(1); // n threads
	spinner.start();
	std::vector<geometry_msgs::Point32> v;
	geometry_msgs::Polygon actr_points;
	v.clear();

	geometry_msgs::Point32 p1;
	p1.x=-4.75;
	p1.y=2.1;
	p1.z=1;
	v.push_back(p1);
	geometry_msgs::Point32 p2;
	p2.x=.4;
	p2.y=4.8;
	p2.z=1;
	v.push_back(p2);
	geometry_msgs::Point32 p3;
	p3.x=-9.3;
	p3.y=1;
	p3.z=1;
	v.push_back(p3);
	geometry_msgs::Point32 p4;
	p4.x=-10.4;
	p4.y=-2.9;
	p4.z=1;
	v.push_back(p4);
	actr_points.points=v;
	pub.publish(actr_points);
	while(n.ok()){

		pub.publish(actr_points);
		loop_rate.sleep();
	/*	v.clear();
		while(true){
			geometry_msgs::Point32 p;
			std::cout<<"x"<<std::endl;
			std::cin>>p.x;
			std::cout<<"y"<<std::endl;
			std::cin>>p.y;
			std::cout<<"intensita"<<std::endl;
			std::cin>>p.z;
			v.push_back(p);
			std::cout<<"continare? [s/n]"<<std::endl;
			char risp;
			std::cin>>risp;
			if(risp=='n') break;
		}
		actr_points.points=v;*/
	}


	return 0;
}


