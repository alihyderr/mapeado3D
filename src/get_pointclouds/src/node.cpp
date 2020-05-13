#include <ros/ros.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// msgs
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

void simpleVis()
{
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	while (!viewer.wasStopped())
	{
		viewer.showCloud(visu_pc);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}
*/

//void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg, const sensor_msgs::JoyConstPtr &twistStamped)
void callback(const sensor_msgs::PointCloud2ConstPtr &msg, const sensor_msgs::JoyConstPtr &twistStamped)
{
	pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
	pcl::fromROSMsg(*msg, pointcloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(pointcloud));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cout << "Puntos capturados: " << cloud->size() << std::endl;

	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud(cloud);
	vGrid.setLeafSize(0.05f, 0.05f, 0.05f);
	vGrid.filter(*cloud_filtered);

	std::cout << "Puntos tras VG: " << cloud_filtered->size() << std::endl;

	//visu_pc = cloud_filtered;
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

	//boost::thread t(simpleVis);

	while (ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}
