#include "autoaim.h"
#include "ros/ros.h"

int main(int argc, char* argv[]){
	ros::init(argc,argv,"autoaim");
	ros::NodeHandle n;

	AutoAim a = AutoAim(&n);
	ros::spin();
}
