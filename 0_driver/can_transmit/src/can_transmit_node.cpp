#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>
#include <string>

#define CAN_NVIDIA_TX2_BOARD_ID 0x103

ros::Publisher can_publisher;
ros::Subscriber cmd_vel_subscriber;

std::string cmd_topic;

// void cmd_cb(const geometry_msgs::Twist &t){
void cmd_cb(const geometry_msgs::Vector3 &t){
	static can_msgs::Frame f;

	// ROS_INFO("Received cmd_vel vx=%f vy=%f vw=%f",t.linear.x,t.linear.y,t.angular.z);
	//f.header.frame_id="0";
	f.header.stamp = ros::Time::now();

	f.id = CAN_NVIDIA_TX2_BOARD_ID;
	f.dlc = (16 / 8) * 3;

	// int16_t vx = (int16_t) (t.linear.x * 100000); // convert to mm/s
	int16_t vx = (int16_t) (t.z * 100000); // convert to mm/s
	int16_t vy = (int16_t) (t.x * 100000); // convert to mm/s
	int16_t vw = (int16_t) (t.y * 100000); // convert to mm/s
	f.data[0] = (uint8_t) (vx >> 8) & 0xff;
	f.data[1] = (uint8_t) vx & 0xff;
	
	// int16_t vy = (int16_t) (-t.linear.y * 100000); // convert to mm/s
	f.data[2] = (uint8_t) (vy >> 8) & 0xff;
	f.data[3] = (uint8_t) vy & 0xff;

	// int16_t vw = (int16_t) -t.angular.z*100000;
	f.data[5] = (uint8_t) (vw >> 8) & 0xff;
	f.data[4] = (uint8_t) vw & 0xff;

	can_publisher.publish(f);
}

int main(int argc, char* argv[]){
    ros::init(argc,argv,"can_transmit_node");
    ros::NodeHandle nh("~");

	nh.param("cmd_topic", cmd_topic, std::string("/cmd_vel"));

	can_publisher = nh.advertise<can_msgs::Frame>("/sent_messages",10);
	cmd_vel_subscriber = nh.subscribe(cmd_topic,10,cmd_cb);
	
	ROS_INFO("CAN transmission started");

    ros::spin();
}
