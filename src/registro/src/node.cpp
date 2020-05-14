#include <ros/ros.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
// sift
#include <pcl/keypoints/sift_keypoint.h>
// fpfh
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

void computeSiftKeypoints( pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, const long unsigned int& file_id, const unsigned int visualization = 0 ) {
	// Parameters for sift computation
	const float min_scale = 0.1f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 10;
	const float min_contrast = 0.5f;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd0.pcd", *cloud);

	// Estimate the sift interest points using Intensity values from RGB values
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(result);

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *keypoints);

	if ( visualization != 0 ) {

		std::cout << "Resulting sift points are of size: " << keypoints->points.size () <<std::endl;
		pcl::io::savePCDFileASCII("sift_points.pcd", *keypoints);

		// Visualization of keypoints along with the original cloud
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(cloud, "cloud");
		viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		while(!viewer.wasStopped ()) {
			viewer.spinOnce ();
		}
	}
}

void computeFPFHFeatures( pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints ) {
	std::cout << "Loaded " << keypoints->points.size () << " points." << std::endl;
	
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud (keypoints);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

	normal_estimation.setRadiusSearch (0.03);

	normal_estimation.compute (*cloud_with_normals);

	// Setup the feature computation

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	// Provide the original point cloud (without normals)
	fpfh_estimation.setInputCloud (keypoints);
	// Provide the point cloud with normals
	fpfh_estimation.setInputNormals (cloud_with_normals);

	// fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
	// Use the same KdTree from the normal estimation
	fpfh_estimation.setSearchMethod (tree);

	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);

	fpfh_estimation.setRadiusSearch (0.2);

	// Actually compute the spin images
	fpfh_estimation.compute (*pfh_features);

	std::cout << "output points.size (): " << pfh_features->points.size () << std::endl;

	// Display and retrieve the shape context descriptor vector for the 0th point.
	pcl::FPFHSignature33 descriptor = pfh_features->points[0];
	std::cout << descriptor << std::endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "registro");

	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_keypoints ( new pcl::PointCloud<pcl::PointXYZ>() );
	computeSiftKeypoints( sift_keypoints, 1, false );
	
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	computeFPFHFeatures( pfh_features, sift_keypoints );

	

	return 0;
}
