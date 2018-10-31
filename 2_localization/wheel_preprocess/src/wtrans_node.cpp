#include "ros/ros.h"
#include "std_msgs/String.h"
#include <can_receive_msg/motor_debug.h>
#include <wheel_odom/wheelSpeeds.h>

#define CON_PI_F 3.14159265359
#define CON_RPM_TO_RAD ( 2.0 * CON_PI_F / 60.0 )
#define GEAR_SCALE 1

ros::Publisher pub;

double R;

void
chatterCallback( const can_receive_msg::motor_debugConstPtr& msg )
{
    wheel_odom::wheelSpeeds speeds;

    speeds.header.frame_id = "wheel";
    speeds.header.stamp    = msg->header.stamp;
    speeds.header.seq      = msg->header.seq;

    speeds.speedRF = GEAR_SCALE * CON_RPM_TO_RAD * msg->speed_[0] * R;
    speeds.speedLF = GEAR_SCALE * CON_RPM_TO_RAD * msg->speed_[1] * R;
    speeds.speedLB = GEAR_SCALE * CON_RPM_TO_RAD * msg->speed_[2] * R;
    speeds.speedRB = GEAR_SCALE * CON_RPM_TO_RAD * msg->speed_[3] * R;

    pub.publish( speeds );
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "wtrans_node" );
    ros::NodeHandle nh( "~" );

    double R_tmp = 0.076;
    int freq     = 0;
    nh.getParam( "radius_wheel", R_tmp );
    nh.getParam( "freq", freq );

    std::cout << " radius_wheel " << R_tmp << "\n";
    R = R_tmp;

    ros::Subscriber sub = nh.subscribe( "/can_receive_1/motor_debug", 1000, chatterCallback );
    pub                 = nh.advertise< wheel_odom::wheelSpeeds >( "/wheelSpeeds", 50 );

    ros::spin( );

    return 0;
}
