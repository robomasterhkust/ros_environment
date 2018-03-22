#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

static geometry_msgs::Twist msg;

void key_Callback(const geometry_msgs::Twist& key_msg)
{
	if(key_msg.linear.x > 0.0)
		msg.angular.y = 0.5;
	else if(key_msg.linear.x < 0.0)
		msg.angular.y = -0.5;
	else
		msg.angular.y = 0.0;

	msg.angular.z = key_msg.angular.z / 2.0;
}

int main(int argc, char *argv[]){
	ros::init(argc,argv,"fakeCV");
	ros::NodeHandle n;

	ros::Publisher  twist_pub = n.advertise<geometry_msgs::Twist>("cv_result",100);
	ros::Subscriber key_sub = n.subscribe("/key_vel",100, key_Callback);
	ros::Rate loop_rate(50);

	memset(&msg, 0, sizeof(geometry_msgs::Twist));

	while(ros::ok())
	{
		twist_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
