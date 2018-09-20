#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <thread>

#include "../ptgrey_lib/camera.h"
//#include "../bluefox2/camera.h"
//#include <djiros/DjiRos.h>
#include "../../include/djiros/DjiRos.h"

// #define BACKWARD_HAS_BFD 1
// #include "backward.hpp"

// namespace backward {
// backward::SignalHandling sh;
// }

bool g_shutdown_required;
sighandler_t g_last_signal_handler;

void
mySigintHandler( int sig )
{
    if ( g_last_signal_handler )
    {
        g_last_signal_handler( sig );
    }
    g_shutdown_required = true;
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "djiros" );
    ros::NodeHandle nh;
    ros::NodeHandle nh_private( "~" );

    auto djiros = std::make_shared< DjiRos >( nh, nh_private );

    ptgrey::Camera camera( nh_private );
    std::shared_ptr< HardwareSynchronizer > hwsync( new HardwareSynchronizer( ) );

    djiros->m_hwsync = hwsync;
    camera.m_hwsync  = hwsync;

    std::thread cam_thread;
    if ( camera.is_slave_mode( ) )
    {
        //        if ( camera.is_fast_mode( ) )
        //        {
        //            ROS_WARN( "[djiros/cam] camera work in fast mode" );
        //            cam_thread = std::thread( &ptgrey::Camera::process_fast_sync,
        //            &camera );
        //        }
        //        else
        {
            cam_thread = std::thread( &ptgrey::Camera::process_slow_sync, &camera );
        }
    }
    else
    {
        cam_thread = std::thread( &ptgrey::Camera::feedImages, &camera );
    }

    //    ros::AsyncSpinner spinner(4); // Use 4 threads
    //    spinner.start();

    g_shutdown_required   = false;
    g_last_signal_handler = signal( SIGINT, mySigintHandler );

    ros::Rate r( 200.0 );
    while ( ros::ok( ) && !g_shutdown_required )
    {
        ros::spinOnce( );
        djiros->process( );
        r.sleep( );
    }

    ROS_ERROR( "[djifox] Exit..." );
    cam_thread.join( );
    ros::shutdown( );

    return 0;
}
