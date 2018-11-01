#include "WheelOdometryROS/WheelOdometryROS.h"
#include <eigen3/Eigen/Eigen>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "WheelOdometry" );
    ros::NodeHandle n;

    wheel_odom::WheelOdometryROS odom( n, 0.307, 0.419 );

    ros::spin( );

    return 0;
}
