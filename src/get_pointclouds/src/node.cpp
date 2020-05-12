#include <ros/ros.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
// synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// msgs
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud.h>

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

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &msg, const geometry_msgs::TwistStampedConstPtr &twistStamped)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud(cloud);
	vGrid.setLeafSize(0.05f, 0.05f, 0.05f);
	vGrid.filter(*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;

	visu_pc = cloud_filtered;
}

class ImageConverter
{
private:
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::PointCloud> sub_image;
	message_filters::Subscriber<geometry_msgs::TwistStamped> sub_twist;
	typedef message_filters::sync_policies::ApproximateTime<pcl::PointXYZRGB, geometry_msgs::TwistStamped> MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

public:
	ImageConverter()
	{
		sub_image.subscribe(nh, "robot1/camera/rgb/image_raw", 1);
		sub_twist.subscribe(nh, "chatter", 1);
		sync_.reset(new Sync(MySyncPolicy(10), sub_image, sub_twist));
		//sync_->registerCallback(boost::bind(callback, _1, _2));
	}
};

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "sub_pcl");
	//ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

	//boost::thread t(simpleVis);

	/*while(ros::ok())
  {
	ros::spinOnce();
  }*/
}
