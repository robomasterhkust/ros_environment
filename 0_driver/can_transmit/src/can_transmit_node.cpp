#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>

#define CAN_NVIDIA_TX2_BOARD_ID 0x103
#define CAN_RUNE 0x104

ros::Publisher can_publisher;
ros::Subscriber cmd_vel_subscriber;
ros::Subscriber rune_cmd_subscriber;

std::string cmd_topic;
std::string rune_cmd_topic;

void rune_cb(const geometry_msgs::Twist &t) {
	static can_msgs::Frame f;
	f.header.stamp = ros::Time::now();

  f.id = CAN_RUNE;
  f.dlc = 4;

	int16_t py = (int16_t)(t.angular.y * 1000);  // convert to mm/s
	int16_t pz = (int16_t)(t.angular.z * 1000);  // convert to mm/s

	f.data[1] = (uint8_t)(py >> 8) & 0xff;
	f.data[0] = (uint8_t)py & 0xff;

	f.data[3] = (uint8_t)(pz >> 8) & 0xff;
	f.data[2] = (uint8_t)pz & 0xff;

	can_publisher.publish(f);
}

void cmd_cb(const geometry_msgs::Twist &t) {
  // void cmd_cb(const geometry_msgs::Vector3 &t){
  static can_msgs::Frame f;

  // ROS_INFO("Received cmd_vel py=%f vy=%f
  // vw=%f",t.linear.x,t.linear.y,t.angular.z);
  // f.header.frame_id="0";
  f.header.stamp = ros::Time::now();

  f.id = CAN_NVIDIA_TX2_BOARD_ID;
  f.dlc = (16 / 8) * 4;

  int16_t px = (int16_t)(t.linear.x * 1000);  // convert to mm/s
  int16_t py = (int16_t)(t.linear.y * 1000);  // convert to mm/s
  int16_t vy = (int16_t)(t.angular.y * 1000); // pitch, rotate by Y axis
  int16_t vz = (int16_t)(t.angular.z * 1000); // yaw,   rotate by Z axis
  // int16_t py = (int16_t) (t.z * 100000); // convert to mm/s
  // int16_t vy = (int16_t) (t.x * 100000); // convert to mm/s
  // int16_t vw = (int16_t) (t.y * 100000); // convert to mm/s
  f.data[1] = (uint8_t)(py >> 8) & 0xff;
  f.data[0] = (uint8_t)py & 0xff;

  f.data[3] = (uint8_t)(pz >> 8) & 0xff;
  f.data[2] = (uint8_t)pz & 0xff;

  f.data[5] = (uint8_t)(vy >> 8) & 0xff;
  f.data[4] = (uint8_t)vy & 0xff;

  f.data[7] = (uint8_t)(vz >> 8) & 0xff;
  f.data[6] = (uint8_t)vz & 0xff;

  can_publisher.publish(f);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "can_transmit_node");
  ros::NodeHandle nh("~");

  nh.param("cmd_topic", cmd_topic, std::string("/cmd_vel"));
  nh.param("rune_cmd_topic", rune_cmd_topic, std::string("/rune_cmd"));

  can_publisher = nh.advertise<can_msgs::Frame>("/sent_messages", 10);
  cmd_vel_subscriber = nh.subscribe(cmd_topic, 10, cmd_cb);
  rune_cmd_subscriber = nh.subscribe(rune_cmd_topic, 10, rune_cb);

  ROS_INFO("CAN transmission started");

  ros::spin();
}
