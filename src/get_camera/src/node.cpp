#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher publisher;

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	std::cout << "asdasd\n";
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg( *msg, cloud_msg );
	publisher.publish( cloud_msg );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camara_titanica");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);
	publisher = nh.advertise<sensor_msgs::PointCloud2>("puntosPublisher", 1);

	while(ros::ok()) {
		ros::spinOnce();
	}

}
