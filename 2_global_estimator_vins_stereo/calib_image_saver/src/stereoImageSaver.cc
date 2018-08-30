#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/filesystem.hpp>
#include <iomanip>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/calib_image_saver/chessboard/Chessboard.h"

ros::Subscriber image_sub;

std::string image_path;
bool is_use_OpenCV = false;
bool is_show       = false;
bool is_save_data  = false;
std::string data_path;
std::string image_name_left  = "left_";
std::string image_name_right = "right_";
cv::Size boardSize;

cv::Size image_size;
int image_count       = 0;
bool is_first_run     = true;
bool is_get_chessbord = false;
ros::Time time_last, time_now;
int max_freq = 10;
cv::Mat image_in_l, image_in_r, image_show_l, image_show_r;
cv::Mat DistributedImage;
std::vector< std::vector< cv::Point2f > > total_image_points_left;
std::vector< std::vector< cv::Point2f > > total_image_points_right;

void
showImage( cv::Mat& image, cv::Mat& image1, cv::Mat& _DistributedImage )
{
    if ( image.channels( ) == 1 )
        cv::cvtColor( image, image_show_l, CV_GRAY2RGB );
    else
        image_show_l = image;

    if ( image1.channels( ) == 1 )
        cv::cvtColor( image1, image_show_r, CV_GRAY2RGB );
    else
        image_show_r = image1;

    cv::Mat imgROI = _DistributedImage( cv::Rect( 0, image.rows, image.cols, image.rows ) );
    cv::Mat imgROI1 = _DistributedImage( cv::Rect( image.cols, image.rows, image.cols, image.rows ) );
    image_show_l.copyTo( imgROI );
    image_show_r.copyTo( imgROI1 );

    cv::namedWindow( "DistributedImage", cv::WINDOW_NORMAL );
    cv::imshow( "DistributedImage", DistributedImage );
    cv::waitKey( 1000 / max_freq );
}

void
drawChessBoard( cv::Mat& image_input, cv::Mat& _DistributedImage, const std::vector< cv::Point2f >& imagePoints )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );

    cv::Mat& image = image_input;

    if ( image.channels( ) == 1 )
    {
        cv::cvtColor( image, image, CV_GRAY2RGB );
    }

    for ( size_t j = 0; j < imagePoints.size( ); ++j )
    {
        cv::Point2f pObs = imagePoints.at( j );

        // green points is the observed points
        cv::circle( image,
                    cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5,
                    green,
                    2,
                    CV_AA,
                    drawShiftBits );

        // yellow points is the observed points
        cv::circle( _DistributedImage,
                    cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5,
                    yellow,
                    2,
                    CV_AA,
                    drawShiftBits );
    }

    cv::line( _DistributedImage, imagePoints.at( 0 ), imagePoints.at( boardSize.width - 1 ), green, 1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ),
              imagePoints.at( 0 ),
              green,
              1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ),
              imagePoints.at( boardSize.width * boardSize.height - 1 ),
              green,
              1 );
    cv::line( _DistributedImage,
              imagePoints.at( boardSize.width * boardSize.height - 1 ),
              imagePoints.at( boardSize.width - 1 ),
              green,
              1 );
}

void
imageProcessCallback( const sensor_msgs::ImageConstPtr& left_image_msg,
                      const sensor_msgs::ImageConstPtr& right_image_msg )
{
    image_in_l = cv_bridge::toCvCopy( left_image_msg, "mono8" )->image;
    image_in_r = cv_bridge::toCvCopy( right_image_msg, "mono8" )->image;
    time_now   = left_image_msg->header.stamp;

    if ( is_first_run )
    {
        time_last         = left_image_msg->header.stamp;
        image_size.height = left_image_msg->height;
        image_size.width  = left_image_msg->width;

        cv::Mat DistributedImage_tmp( cv::Size( image_size.width * 2, image_size.height * 2 ),
                                      CV_8UC3,
                                      cv::Scalar( 0 ) );

        DistributedImage_tmp.copyTo( DistributedImage );
        is_first_run = false;

        if ( is_show )
            cv::namedWindow( "DistributedImage", CV_WINDOW_NORMAL );
    }

    if ( is_show )
        showImage( image_in_l, image_in_r, DistributedImage );
}

void
process( )
{
    if ( is_first_run )
        return;

    cv::Mat image_left  = image_in_l;
    cv::Mat image_right = image_in_r;

    camera_model::Chessboard chessboard_left( boardSize, image_left );
    camera_model::Chessboard chessboard_right( boardSize, image_right );
#pragma omp parallel sections
    {
#pragma omp section
        {
            chessboard_left.findCorners( is_use_OpenCV );
        }
#pragma omp section
        {
            chessboard_right.findCorners( is_use_OpenCV );
        }
    }

    if ( chessboard_left.cornersFound( ) && chessboard_right.cornersFound( ) )
    {
        std::stringstream ss_num;
        ss_num << image_count;
        std::string image_file_left
        = image_path + "/" + image_name_left + ss_num.str( ) + ".jpg";
        std::string image_file_right
        = image_path + "/" + image_name_right + ss_num.str( ) + ".jpg";

        std::cout << "#[INFO] Get chessboard image, left: "
                  << image_name_left + ss_num.str( ) + ".jpg" << std::endl;
        std::cout << "                              right: "
                  << image_name_right + ss_num.str( ) + ".jpg" << std::endl;

        cv::imwrite( image_file_left, image_left );
        cv::imwrite( image_file_right, image_right );

        ++image_count;
        total_image_points_left.push_back( chessboard_left.getCorners( ) );
        total_image_points_right.push_back( chessboard_right.getCorners( ) );

        if ( is_show )
        {
            cv::Mat DistributedImage_l
            = DistributedImage( cv::Rect( 0, 0, image_left.cols, image_left.rows ) );
            cv::Mat DistributedImage_r
            = DistributedImage( cv::Rect( image_left.cols, 0, image_left.cols, image_left.rows ) );

            drawChessBoard( image_left, DistributedImage_l, total_image_points_left.back( ) );
            drawChessBoard( image_right, DistributedImage_r, total_image_points_right.back( ) );
            showImage( image_left, image_right, DistributedImage );
        }
        is_get_chessbord = true;
    }
    else
    {
        std::cout << "#[ERROR] Get no chessboard image." << std::endl;
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "stereoImageSaver" );
    ros::NodeHandle n( "~" );

    n.getParam( "rate", max_freq );
    n.getParam( "image_path", image_path );
    n.getParam( "board_width", boardSize.width );
    n.getParam( "board_height", boardSize.height );
    n.getParam( "is_use_OpenCV", is_use_OpenCV );
    n.getParam( "is_show", is_show );
    n.getParam( "is_save_data", is_save_data );
    n.getParam( "data_path", data_path );
    n.getParam( "image_name_left", image_name_left );
    n.getParam( "image_name_right", image_name_right );

    if ( !boost::filesystem::exists( image_path ) && !boost::filesystem::is_directory( image_path ) )
    {
        std::cerr << "#[ERROR] Cannot find Saving directory: " << image_path << "." << std::endl;
        return 1;
    }

    if ( boardSize.height <= 1 || boardSize.width <= 1 )
    {
        std::cout << "#[ERROR] Error with input chessbopard Size." << std::endl;
        return 0;
    }

    std::string data_file_name;
    if ( is_save_data )
    {
        if ( data_path.empty( ) )
            data_path = image_path;

        data_file_name = data_path + "/data.ymal";
    }

    message_filters::Subscriber< sensor_msgs::Image > sub_imgL( n, "/left_image", 1 );
    message_filters::Subscriber< sensor_msgs::Image > sub_imgR( n, "/right_image", 1 );

    typedef message_filters::sync_policies::ExactTime< sensor_msgs::Image, sensor_msgs::Image > SyncPolicy;
    // typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,
    // sensor_msgs::Image > SyncPolicy;
    message_filters::Synchronizer< SyncPolicy > sync( SyncPolicy( 50 ), sub_imgL, sub_imgR );

    sync.registerCallback( boost::bind( &imageProcessCallback, _1, _2 ) );

    ros::Rate loop( max_freq );
    while ( ros::ok( ) )
    {
        process( );
        ros::spinOnce( );
        loop.sleep( );
    }
    //    ros::spin( );

    cv::imwrite( image_path + "/" + "Distributed.jpg", DistributedImage );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
