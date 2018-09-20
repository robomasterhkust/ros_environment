#include "camera.h"

ptgrey::Camera::Camera( ros::NodeHandle param_nh )
: pnode( param_nh )
{
    bool is_pub;
    bool is_print;
    bool is_auto_shutter;
    bool is_sync;
    double brightness;
    double exposure;
    double gain;
    double frameRate;
    double shutter;
    int cam_cnt;

    pnode.getParam( "is_pub", is_pub );
    pnode.getParam( "is_print", is_print );
    pnode.getParam( "is_auto_shutter", is_auto_shutter );
    pnode.getParam( "is_sync", is_sync );
    pnode.getParam( "brightness", brightness );
    pnode.getParam( "exposure", exposure );
    pnode.getParam( "gain", gain );
    pnode.getParam( "frameRate", frameRate );
    pnode.getParam( "shutter", shutter );
    pnode.getParam( "cam_cnt", cam_cnt );

    std::vector< unsigned int > IDs;

    m_fps = frameRate;
    images_tmp.resize( cam_cnt );

    image_pubs.clear( );
    for ( int i = 0; i < cam_cnt; i++ )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % i );
        int serialNum;
        std::string topic;

        pnode.getParam( prefix + "serialNum", serialNum );
        pnode.getParam( prefix + "topic", topic );

        unsigned int cameraId = serialNum;
        IDs.push_back( cameraId );

        ros::Publisher imagePublisher = pnode.advertise< sensor_msgs::Image >( topic, 3 );
        image_pubs.push_back( imagePublisher );
    }

    camReader = new ptgrey_reader::multiCameraReader( IDs );

    bool is_cameraStarted = camReader->startCamera( IDs, frameRate, brightness, exposure, gain,
                                                    is_auto_shutter, shutter, is_print, is_sync );

    m_is_color = camReader->Cameras( )->isColorCamera( );
    if ( !is_cameraStarted )
    {
        ros::shutdown( );
        std::cout << "[#INFO] Camera cannot start" << std::endl;
    }
}

void
ptgrey::Camera::feedImages( )
{
    ros::Rate r( m_fps );
    while ( pnode.ok( ) )
    {
        if ( camReader->grabImage( images_tmp ) )
        {
            for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp.sec = images_tmp.at( pub_index ).time.seconds;
                outImg.header.stamp.nsec = images_tmp.at( pub_index ).time.microSeconds * 1000;

                outImg.header.frame_id = "frame";
                if ( m_is_color )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
                {
                    outImg.image = images_tmp.at( pub_index ).image;
                    image_pubs.at( pub_index ).publish( outImg );
                }
            }
        }
        ros::spinOnce( );
        r.sleep( );
    }
}

bool
ptgrey::Camera::wait_for_imu_ack( SyncAckInfo& sync_ack, int& queue_size )
{
    ros::Time wait_start_time = ros::Time::now( );
    while ( 1 )
    {
        if ( !pnode.ok( ) )
        {
            return false;
        }

        // Check imu response
        {
            std::lock_guard< std::mutex > lg( m_hwsync->ack_mutex );
            queue_size = m_hwsync->ack_queue.size( );
            if ( m_hwsync->ack_queue.size( ) )
            {
                if ( m_hwsync->ack_queue.size( ) == 1 )
                {
                    // ROS_INFO("ack queue size = %zu", m_hwsync->ack_queue.size());
                }
                else
                {
                    ROS_ERROR( "ack queue size = %zu", m_hwsync->ack_queue.size( ) );
                    ROS_ERROR(
                    "Cannot sync! Image capturing is too slow! Try to decrease "
                    "fps/exposure or "
                    "turn off aec!" );
                    // ros::shutdown();
                }
                // get first and erase it
                sync_ack = m_hwsync->ack_queue.front( );
                m_hwsync->ack_queue.pop( );

                // return the ack
                break;
            };
        }

        // sleep for the while loop
        ros::Duration( 5.0 / 1000.0 ).sleep( );

        ros::Duration dt = ros::Time::now( ) - wait_start_time;

        // Timeout set as two times longer than normal interval
        if ( dt.toSec( ) > ( 1.0 / m_fps * 2 ) )
        {
            ROS_WARN_THROTTLE( 1.0, "Wait %.3f secs for imu ack", dt.toSec( ) );
            return false;
        }
    }

    return true;
}

void
ptgrey::Camera::process_slow_sync( )
{
    ROS_ASSERT( m_hwsync.get( ) );

    ros::Rate r( m_fps );
    while ( pnode.ok( ) )
    {
        // Wait for imu ack
        SyncAckInfo sync_ack;
        int queue_size = 0;
        if ( !wait_for_imu_ack( sync_ack, queue_size ) )
        {
            ROS_ERROR( "reset all driver requests!" );
            goto END_OF_OUT_LOOP;
        }

        // Imu ack received, verify it
        ROS_ASSERT( sync_ack.seq >= 0 );

        // Get image from driver
        if ( camReader->grabImage( images_tmp ) )
        {
            ROS_INFO_COND( m_verbose_output, "Grab data with seq[%d]", sync_ack.seq );
            ROS_ASSERT_MSG( sync_ack.seq == m_hwsync_grab_count,
                            "sync_ack.seq=%d, hwsync_grab_count=%d", sync_ack.seq, m_hwsync_grab_count );

            capture_time = ros::Time( sync_ack.stamp );

            for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
            {
                cv_bridge::CvImage outImg;
                outImg.header.stamp.sec = images_tmp.at( pub_index ).time.seconds;
                outImg.header.stamp.nsec = images_tmp.at( pub_index ).time.microSeconds * 1000;

                outImg.header.frame_id = "frame";
                if ( m_is_color )
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
                else
                    outImg.encoding = sensor_msgs::image_encodings::MONO8;

                for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
                {
                    outImg.image = images_tmp.at( pub_index ).image;
                    image_pubs.at( pub_index ).publish( outImg );
                }
            }
        }

        m_hwsync_grab_count++; // inc it whether grab success or failed.

    END_OF_OUT_LOOP:
        r.sleep( );

        if ( m_max_req_number > 0 && m_hwsync_grab_count >= m_max_req_number )
        {
            break;
        }
    }
}

bool
ptgrey::Camera::isOK( )
{
    return ok;
}

bool
ptgrey::Camera::is_slave_mode( ) const
{
    return m_is_slave_mode;
}

bool
ptgrey::Camera::is_fast_mode( ) const
{
    return m_fast_mode;
}
