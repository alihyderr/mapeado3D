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
// correspondencias
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// transformación
#include <pcl/registration/transformation_estimation_svd.h>

void computeSiftKeypoints(
	pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints, 
	const size_t& file_id, 
	const size_t& visualization = 0 )
{
	// Parameters for sift computation
	const float min_scale = 0.1f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 10;
	const float min_contrast = 0.5f;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("test_pcd" + std::to_string( file_id ) + ".pcd", *cloud);

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
		//std::cout << "Resulting sift points are of size: " << keypoints->points.size () <<std::endl;
		//pcl::io::savePCDFileASCII("sift_points.pcd", *keypoints);

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

void computeFPFHFeatures(
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& pfh_features, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints ) 
{
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

void findCorrespondences(
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_src,
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_tgt,
    pcl::Correspondences& all_correspondences) {

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(fpfh_src);
    est.setInputTarget(fpfh_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}

void rejectCorrespondences( 
	pcl::Correspondences& remaining_correspondences,
	const pcl::CorrespondencesPtr& correspondences,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& sift_keypoints1,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& sift_keypoints2) {

    // copy only XYZRGB data of keypoints for use in estimating features
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_src(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_tgt(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*sift_keypoints1, *keypoints_src);
    pcl::copyPointCloud(*sift_keypoints2, *keypoints_tgt);

    // RandomSampleConsensus bad correspondence rejector
    pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointXYZRGB> rejector;
    rejector.setInputSource (keypoints_src);
    rejector.setInputTarget (keypoints_tgt);
    rejector.setInlierThreshold(0.1);
    rejector.setMaximumIterations(5000);
    rejector.setRefineModel(true);
    rejector.setInputCorrespondences(correspondences);
    rejector.getCorrespondences(remaining_correspondences);	
}

void correspondancesViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& sift_keypoints1, 
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& sift_keypoints2, 
			const pcl::CorrespondencesPtr& remaining_correspondences,
			const size_t id1,
			const size_t id2) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd" + std::to_string( id1 ) + ".pcd", *src);
	pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd" + std::to_string( id2 ) + ".pcd", *tgt);

	pcl::visualization::PCLVisualizer corresp_viewer("Correspondences Viewer");
    corresp_viewer.setBackgroundColor(0, 0, 0);
    corresp_viewer.addPointCloud(src, "Src cloud");
    corresp_viewer.addPointCloud(tgt, "Tgt cloud");
    corresp_viewer.addPointCloud<pcl::PointXYZ>(sift_keypoints1, "src");
    corresp_viewer.addPointCloud<pcl::PointXYZ>(sift_keypoints2, "tgt");
    corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "src");
    corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "tgt");

    for (auto i = 0; i < remaining_correspondences->size(); ++i) 
	{
        pcl::PointXYZ& src_idx = src->points[(*remaining_correspondences)[i].index_query];
        pcl::PointXYZ& tgt_idx = tgt->points[(*remaining_correspondences)[i].index_match];
        std::string lineID =  std::to_string(i);
        std::string lineID2 = std::to_string(i+700);

        corresp_viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 0.965116, 1, 0.895349, lineID);
    }

	while (!corresp_viewer.wasStopped()) {
			corresp_viewer.spinOnce();
	}

	corresp_viewer.close();
}


void computeIteration(
	const size_t& i,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& M,
	Eigen::Matrix4f& Tt)
{
	// Pasos 4 y 5: Obtener características de Ct Ct+1
	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_keypoints1 ( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_keypoints2 ( new pcl::PointCloud<pcl::PointXYZ>() );
	computeSiftKeypoints( sift_keypoints1, i, false );
	computeSiftKeypoints( sift_keypoints2, i+1, false );
	
	// Descriptores de las características con PFPH
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfph_features1 (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfph_features2 (new pcl::PointCloud<pcl::FPFHSignature33>);
	computeFPFHFeatures( pfph_features1, sift_keypoints1 );
	computeFPFHFeatures( pfph_features2, sift_keypoints2 );
	
	// Paso 6: Obtener emparejamientos
	boost::shared_ptr<pcl::Correspondences> correspondences ( new pcl::Correspondences() );
	findCorrespondences( pfph_features1, pfph_features2, *correspondences );
	std::cout << "correspondences size: " << correspondences->size() << '\n';
	// eliminar correspondencias no válidas con RANSAC
	pcl::CorrespondencesPtr remaining_correspondences ( new pcl::Correspondences() );
	rejectCorrespondences( *remaining_correspondences, correspondences, sift_keypoints1, sift_keypoints2 );
	std::cout << "remaining_correspondences size: " << remaining_correspondences->size() << '\n';
	correspondancesViewer(sift_keypoints1, sift_keypoints2, remaining_correspondences, i, i+1);
	
	// Paso 7: Obtener las mejor transformación Ti
	Eigen::Matrix4f Ti;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
	svd.estimateRigidTransformation(*sift_keypoints1, *sift_keypoints2, *remaining_correspondences, Ti);
	std::cout << "mejor matriz transformacion:\n" << Ti << std::endl;	

	// Paso 8: Obtener la transformación total
	Tt = Tt * Ti;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "registro");

	// Mapa 3D
	pcl::PointCloud<pcl::PointXYZ>::Ptr M( new pcl::PointCloud<pcl::PointXYZ>() );

	// Tt = I
	Eigen::Matrix4f Tt;
	Tt << 
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	std::cout << Tt;

	const size_t t = 9;

	for ( auto i = 1; i < t; ++i ) {
		computeIteration( i, M, Tt );
	}

	std::cout << "Tt\n" << Tt;

	return 0;
}
