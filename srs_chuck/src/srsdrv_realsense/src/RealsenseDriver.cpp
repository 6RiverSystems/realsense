#include <RealsenseDriver.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
RealsenseDriver::RealsenseDriver() :
	rosNodeHandle_(),
	infrared1Subscriber_( rosNodeHandle_.subscribe("/camera/infrared1/image_raw", 10, &RealsenseDriver::onInfrared1, this) ),
	infrared2Subscriber_( rosNodeHandle_.subscribe("/camera/infrared2/image_raw", 10, &RealsenseDriver::onInfrared2, this) ),
	pointCloudSubscriber_( rosNodeHandle_.subscribe("/camera/depth/points", 10, &RealsenseDriver::onPointCloud, this) ),
	infrared1Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>("/camera/infrared1/image_near", 10 ) ),
	infrared2Publisher_( rosNodeHandle_.advertise<sensor_msgs::Image>("/camera/infrared2/image_near", 10 ) )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RealsenseDriver::run() {
	ros::Rate refreshRate(REFRESH_RATE_HZ);

	while (ros::ok()) {
		ros::spinOnce();

		refreshRate.sleep();
	}
}

void RealsenseDriver::onInfrared1( const sensor_msgs::Image& infraredImage1 )
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(infraredImage1, sensor_msgs::image_encodings::TYPE_8UC1);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Apply a binary threshold filter to remove all but the saturated pixels
	cv::threshold( cv_ptr->image, cv_ptr->image, 254, 128, cv::THRESH_BINARY );

	infrared1Publisher_.publish( cv_ptr->toImageMsg( ) );
}

void RealsenseDriver::onInfrared2( const sensor_msgs::Image& infraredImage2 )
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(infraredImage2, sensor_msgs::image_encodings::TYPE_8UC1);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Apply a binary threshold filter to remove all but the saturated pixels
	cv::threshold( cv_ptr->image, cv_ptr->image, 254, 128, cv::THRESH_BINARY );

	infrared2Publisher_.publish( cv_ptr->toImageMsg( ) );
}

void RealsenseDriver::onPointCloud( const sensor_msgs::PointCloud2& pointCloud )
{
	sensor_msgs::PointCloud2 cloud_filtered;

	// Create the filtering object
//	pcl::PassThrough<sensor_msgs::PointCloud2> pass;
//	pass.setInputCloud( pointCloud );
//	pass.setFilterFieldName( "z" );
//	pass.setFilterLimits( 0.0, 1.0 ); // unit : meter
//	pass.filter( cloud_filtered );
//	std::cout << "PointCloud after filtering : " << cloud_filtered.width * cloud_filtered.height << std::endl;
//
//	// Publish the data
//	pub.publish (cloud_filtered);
//
//
//		// Create the filtering object
//		pcl::PassThrough<pcl::PointXYZ> pass;
//		pass.setInputCloud( pcl_pc2 );
//		pass.setFilterFieldName( "z" );
//		pass.setFilterLimits( 0.3, 3.5 );
//
//		pass.filter( filteredCloud );
//
//		std::swap( filtered_cloud_, tmp );
//
//		render_cloud_ = filtered_cloud_;
//
//		// Create the filtering object
//		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//		sor.setInputCloud( render_cloud_ );
//		sor.setMeanK( 50 );
//		sor.setStddevMulThresh( 1.0 );
//
//		sor.filter( *tmp );
//
//		std::swap( filtered_cloud_, tmp );
//
//		render_cloud_ = filtered_cloud_;
//
//	pcl::fromPCLPointCloud2( pcl_pc2, filteredCloud );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs
