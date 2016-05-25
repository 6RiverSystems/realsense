#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver( ) :
	rosNodeHandle_( ),
	infrared1Subscriber_( rosNodeHandle_, "/camera/infrared1/image_raw", 10 ),
	infrared2Subscriber_( rosNodeHandle_, "/camera/infrared2/image_raw", 10 ),
	laserScanSubscriber_( rosNodeHandle_, "/camera/scan", 10 ),
	infrared1Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared1/image_near", 10 ) ),
	infrared2Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared2/image_near", 10 ) ),
	infraredPublisher_( rosNodeHandle_.advertise<sensor_msgs::Image>( "/camera/infrared/image_near", 10 ) ),
	synchronizer_( new ImageSyncronizer( laserScanSubscriber_, infrared1Subscriber_, infrared2Subscriber_, 10 ) )
{
	synchronizer_->registerCallback(
		std::bind( &RealsenseDriver::OnDepthData, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RealsenseDriver::run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		refreshRate.sleep( );
	}
}

void RealsenseDriver::OnDepthData( const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::Image::ConstPtr& infraredImage1,
	const sensor_msgs::Image::ConstPtr& infraredImage2 )
{
	cv_bridge::CvImagePtr cvImage1 = GetCvImage( infraredImage1 );

	cv_bridge::CvImagePtr cvImage2 = GetCvImage( infraredImage2 );

	cv::Mat combinedImages;
	CombineImages( cvImage1->image, cvImage2->image, combinedImages );

	//ThresholdImage( cvImage1->image );

	infrared1Publisher_.publish( cvImage1->toImageMsg( ) );

	//ThresholdImage( cvImage2->image );

	infrared2Publisher_.publish( cvImage2->toImageMsg( ) );

	//ThresholdImage( combinedImages );

	cv_bridge::CvImage combinedMsg;
	combinedMsg.header   = infraredImage2->header;
	combinedMsg.encoding = infraredImage2->encoding;
	combinedMsg.image    = combinedImages;

	infraredPublisher_.publish( combinedMsg );
}

cv_bridge::CvImagePtr RealsenseDriver::GetCvImage( const sensor_msgs::Image::ConstPtr& image ) const
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, sensor_msgs::image_encodings::TYPE_8UC1 );
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "cv_bridge exception: %s", e.what( ) );
	}

	return cv_ptr;
}

void RealsenseDriver::ThresholdImage( cv::Mat& image ) const
{
	// Apply a binary threshold filter to remove all but the saturated pixels
	cv::threshold( image, image, 254, 128, cv::THRESH_BINARY );
}

void RealsenseDriver::CombineImages( cv::Mat& image1, cv::Mat& image2, cv::Mat& result ) const
{
	if( image1.cols == image2.cols &&
		image1.rows == image2.rows )
	{
		constexpr uint32_t irOffset = 70;

		// Further optimization: Use stitching algorithm to align images (not sure it's worth the cycles though)
		cv::Mat combined( image1.rows, image1.cols + irOffset, image1.type( ) );

		image1.copyTo( combined( cv::Rect( 0, 0, image1.cols, image1.rows ) ) );
		image2.copyTo( combined( cv::Rect( irOffset, 0, image2.cols, image2.rows ) ) );

		result = combined;
	}
}

//void RealsenseDriver::onInfrared2( const sensor_msgs::Image& infraredImage2 )
//{
//	cv_bridge::CvImagePtr cv_ptr;
//	try {
//		cv_ptr = cv_bridge::toCvCopy(infraredImage2, sensor_msgs::image_encodings::TYPE_8UC1);
//	} catch (cv_bridge::Exception& e) {
//		ROS_ERROR("cv_bridge exception: %s", e.what());
//		return;
//	}
//
//	// Apply a binary threshold filter to remove all but the saturated pixels
//	cv::threshold( cv_ptr->image, cv_ptr->image, 254, 128, cv::THRESH_BINARY );
//
//	infrared2Publisher_.publish( cv_ptr->toImageMsg( ) );
//}
//
//void RealsenseDriver::onPointCloud( const sensor_msgs::PointCloud2ConstPtr& cloud_msg )
//{
////	// build laserscan output
////	sensor_msgs::LaserScan output;
////	output.header = cloud_msg->header;
////
////	output.angle_min = angle_min_;
////	output.angle_max = angle_max_;
////	output.angle_increment = angle_increment_;
////	output.time_increment = 0.0;
////	output.scan_time = scan_time_;
////	output.range_min = range_min_;
////	output.range_max = range_max_;
////
////	//determine amount of rays to create
////	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
////
////	//determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
////	if (use_inf_)
////	{
////	  output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
////	}
////	else
////	{
////	  output.ranges.assign(ranges_size, output.range_max + 1.0);
////	}
////
////	sensor_msgs::PointCloud2ConstPtr cloud_out;
////	sensor_msgs::PointCloud2Ptr cloud;
////
////	// Iterate through pointcloud
////	for (sensor_msgs::PointCloud2ConstIterator<float>
////			  iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
////			  iter_x != iter_x.end();
////			  ++iter_x, ++iter_y, ++iter_z)
////	{
////
////	  if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
////	  {
////		ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
////		continue;
////	  }
////
////	  if (*iter_z > max_height_ || *iter_z < min_height_)
////	  {
////		  ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
////		continue;
////	  }
////
////	  double range = hypot(*iter_x, *iter_y);
////	  if (range < range_min_)
////	  {
////		  ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
////					  *iter_z);
////		continue;
////	  }
////
////	  // http://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
////
////	  double angle = atan2(*iter_y, *iter_x);
////	  if (angle < output.angle_min || angle > output.angle_max)
////	  {
////		  ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
////		continue;
////	  }
////
////	  //overwrite range at laserscan ray if new range is smaller
////	  int index = (angle - output.angle_min) / output.angle_increment;
////	  if (range < output.ranges[index])
////	  {
////		output.ranges[index] = range;
////	  }
////
////	}
////	pub_.publish(output);
////  }
//}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs
