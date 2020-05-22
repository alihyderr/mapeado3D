#include <ros/ros.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <boost/foreach.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// msgs
#include <sensor_msgs/Joy.h>
//#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

int capturas = 1;

//void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg, const sensor_msgs::JoyConstPtr &twistStamped)
void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const sensor_msgs::JoyConstPtr& joy)
{
	if ( joy->axes[ 5 ] == 1 ) {
		std::cout << " nop\n";
	} else if ( joy->axes[ 5 ] <= 0.9 && joy->axes[ 5 ] > 0){
		pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
		pcl::fromROSMsg(*msg, pointcloud);

		pcl::io::savePCDFileASCII( "X" + std::to_string( capturas++ )+ ".pcd", pointcloud );
		std::cerr << "Saved " << pointcloud.points.size () << " data points to " << capturas << std::endl;
	}
}

class ImageConverter
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_image;
	message_filters::Subscriber<sensor_msgs::Joy> sub_twist;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Joy> MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

public:
	ImageConverter()
	{
		sub_image.subscribe(nh, "puntosPublisher", 1);
		sub_twist.subscribe(nh, "chatter", 1);
		sync_.reset(new Sync(MySyncPolicy(10), sub_image, sub_twist));
		sync_->registerCallback(boost::bind(callback, _1, _2));
	}
};

int main(int argc, char **argv) {
	puts("main get_cloud");
	ros::init(argc, argv, "sub_pcl");
	ImageConverter ic;

	while (ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}
