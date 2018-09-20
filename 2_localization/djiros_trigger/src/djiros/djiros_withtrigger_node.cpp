#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <thread>

#include <djiros/DjiRos.h>

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

    int trigger_freq;
    nh_private.getParam( "trigger_freq", trigger_freq );

    auto djiros = std::make_shared< DjiRos >( nh, nh_private );

    std::shared_ptr< HardwareSynchronizer > hwsync( new HardwareSynchronizer( ) );
    djiros->m_hwsync = hwsync;
    djiros->set_continue_trigger( trigger_freq );

    g_shutdown_required   = false;
    g_last_signal_handler = signal( SIGINT, mySigintHandler );

    ros::Rate r( 400.0 );

    while ( ros::ok( ) && !g_shutdown_required )
    {
        ros::spinOnce( );
        djiros->process( );
        r.sleep( );
    }

    ROS_ERROR( "[djiros] Exit..." );
    ros::shutdown( );

    return 0;
}
