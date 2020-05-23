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
#include <pcl/features/pfhrgb.h>
#include <pcl/features/pfh.h>
// correspondencias
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// transformación
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/filters/statistical_outlier_removal.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */

/*
	Parámetros por defecto
*/
// relacionado con los ficheros
const size_t numero_de_nubes = 32;
const std::string raiz = "datos";
const std::string nubes = raiz + "/nubes/";
const std::string parametros = raiz + "/parametros/";
// Parameters for sift computation
const float min_scale 			= 0.01f;
const int 	n_octaves 			= 3;
const int 	n_scales_per_octave = 4;
const float min_contrast 		= 0.001f;
// Parametros FPFH
const double normal_estimation_radius 	= 0.05;
const double fpfh_estimation_radius 	= 0.25;
// reject correspondences
const double rejector_threshold 	= 0.025;
const double rejector_maxIterations = 10000;
const bool	 refineModel 			= true;


// Mapa 3D
pcl::PointCloud<pcl::PointXYZRGB>::Ptr M( new pcl::PointCloud<pcl::PointXYZRGB>() );

void computeSiftKeypoints(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
	const size_t& visualization = 0 )
{
	// Estimate the sift interest points using Intensity values from RGB values
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale>::Ptr result (new pcl::PointCloud<pcl::PointWithScale> ());
	//pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(*result);

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	copyPointCloud(*result, *keypoints);

	// Visualization of keypoints along with the original cloud
	if ( visualization != 0 ) {
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (keypoints, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (cloud, 255, 255, 0);
		viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
		viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
		viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

		while(!viewer.wasStopped ()) {
			viewer.spinOnce ();
		}
	}
}

/*
void computeFPFHFeatures(
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr& pfh_features, 
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube) 
{
*/
void computeFPFHFeatures(
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr& pfh_features, 
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube) 
{
	// si no se hace downsampling no es viable computar fpfh en el portatil
/*	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud(toda);
	vGrid.setLeafSize(0.025f, 0.025f, 0.025f);
	vGrid.filter(*cloud_filtered);
*/
	std::cout << "Loaded " << keypoints->points.size () << " points." << std::endl;

	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normal_estimation;
	normal_estimation.setInputCloud (nube);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	normal_estimation.setSearchMethod (tree);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	normal_estimation.setRadiusSearch (normal_estimation_radius);
	normal_estimation.compute (*cloud_with_normals);

	// Setup the feature computation
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	//pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> estimation;
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> estimation;

	estimation.setInputCloud (keypoints_xyzrgb);
	estimation.setSearchSurface(nube);
	estimation.setInputNormals (cloud_with_normals);
	estimation.setSearchMethod (tree);
	estimation.setRadiusSearch (fpfh_estimation_radius);
	estimation.compute (*pfh_features);
}

/*
void findCorrespondences(
	const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr& fpfh_src,
    const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr& fpfh_tgt,
    pcl::Correspondences& all_correspondences) {
*/
void findCorrespondences(
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_src,
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_tgt,
    pcl::Correspondences& all_correspondences)
{
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(fpfh_src);
    est.setInputTarget(fpfh_tgt);
    est.determineReciprocalCorrespondences(all_correspondences);
}

Eigen::Matrix4f rejectCorrespondences( 
	pcl::Correspondences& remaining_correspondences,
	const pcl::CorrespondencesPtr& correspondences,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sift_keypoints1,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sift_keypoints2) {

    // copy only XYZRGB data of keypoints for use in estimating features
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_src(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_tgt(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*sift_keypoints1, *keypoints_src);
    pcl::copyPointCloud(*sift_keypoints2, *keypoints_tgt);

    // RandomSampleConsensus bad correspondence rejector
    pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointXYZRGB> rejector;
    rejector.setInputSource (keypoints_src);
    rejector.setInputTarget (keypoints_tgt);
    rejector.setInlierThreshold(rejector_threshold);
    rejector.setMaximumIterations(rejector_maxIterations);
    rejector.setRefineModel(refineModel);
    rejector.setInputCorrespondences(correspondences);
    rejector.getCorrespondences(remaining_correspondences);

	return rejector.getBestTransformation();
}

void correspondancesViewer(
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sift_keypoints1, 
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sift_keypoints2, 
	const pcl::CorrespondencesPtr& remaining_correspondences,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& tgt) 
{
	pcl::visualization::PCLVisualizer corresp_viewer("visión titánica");
    corresp_viewer.setBackgroundColor(0, 0, 0);
    //corresp_viewer.addPointCloud(src, "Src cloud");
    //corresp_viewer.addPointCloud(tgt, "Tgt cloud");
    corresp_viewer.addPointCloud<pcl::PointXYZRGB>(sift_keypoints1, "src");
    corresp_viewer.addPointCloud<pcl::PointXYZRGB>(sift_keypoints2, "tgt");
    corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "src");
    corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "tgt");

    for (auto i = 0; i < remaining_correspondences->size(); ++i) {
        pcl::PointXYZRGB& src_idx = src->points[(*remaining_correspondences)[i].index_query];
        pcl::PointXYZRGB& tgt_idx = tgt->points[(*remaining_correspondences)[i].index_match];
        std::string lineID =  std::to_string(i);
        corresp_viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB>(src_idx, tgt_idx, 0.50, 0.50, 0.895349, lineID);
    }

	while (!corresp_viewer.wasStopped()) {
			corresp_viewer.spinOnce();
	}

	corresp_viewer.close();
}


void computeIteration(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& M,
	Eigen::Matrix4f& Tt,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube1,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube2)
{
	// Pasos 4 y 5: Obtener características de Ct Ct+1
	puts("->Características");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sift_keypoints1 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sift_keypoints2 ( new pcl::PointCloud<pcl::PointXYZRGB>() );
	computeSiftKeypoints( sift_keypoints1, nube1, false );
	computeSiftKeypoints( sift_keypoints2, nube2, false );
	std::cout << "Ci.size()   = " << sift_keypoints1->size() << "\n";
	std::cout << "Ci+1.size() = " << sift_keypoints2->size() << "\n";
	
	// Descriptores de las características con PFPH
	puts("->Descriptores");

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfph_features1 (new pcl::PointCloud<pcl::FPFHSignature33> ());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfph_features2 (new pcl::PointCloud<pcl::FPFHSignature33> ());

	//pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfph_features1 (new pcl::PointCloud<pcl::PFHRGBSignature250>);
	//pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfph_features2 (new pcl::PointCloud<pcl::PFHRGBSignature250>);
	computeFPFHFeatures( pfph_features1, sift_keypoints1, nube1 );
	computeFPFHFeatures( pfph_features2, sift_keypoints2, nube2 );
	std::cout << "pfph_i[0] = " << pfph_features1->points[0] << '\n';
	std::cout << "pfph_i+1[0] = " << pfph_features1->points[0] << '\n';
	
	// Paso 6: Obtener emparejamientos
	puts("->Emparejamientos");
	boost::shared_ptr<pcl::Correspondences> correspondences ( new pcl::Correspondences() );
	findCorrespondences( pfph_features1, pfph_features2, *correspondences );
	std::cout << "correspondences.size() = " << correspondences->size() << '\n';

	// eliminar correspondencias no válidas con RANSAC
	puts("->Mejores emparejamientos");
	pcl::CorrespondencesPtr remaining_correspondences ( new pcl::Correspondences());

	Eigen::Matrix4f Taux;
	if ( correspondences->size() != 0) {
		puts("->Matriz trasnformación Taux");
		Taux = rejectCorrespondences( *remaining_correspondences, correspondences, sift_keypoints1, sift_keypoints2 );
		std::cout << Taux << '\n';
		std::cout << "remaining_correspondences.size() = " << remaining_correspondences->size() << '\n';
	} else {
		puts("correspondence.size() == 0, no seguir los pasos");
		return;
	}
	
	//correspondancesViewer(sift_keypoints1, sift_keypoints2, remaining_correspondences, nube1, nube2);
	
	// Paso 7: Obtener las mejor transformación Ti
	puts("->Matriz trasnformación Ti");
	Eigen::Matrix4f Ti;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> svd;
	svd.estimateRigidTransformation(*sift_keypoints1, *sift_keypoints2, *remaining_correspondences, Ti);
	std::cout << Ti << '\n';

	// Paso 8: Obtener la transformación total
	puts("->Matriz trasnformación total Tt");
	Tt = Tt * Taux;
	std::cout << Tt << '\n';

	// Paso 9: Aplicar Tt a Ci+1
	puts("->Aplicar Tt");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*nube1, *cloud_out, Tt);

	*M += *cloud_out;
}

void magia () {
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (M, 0, 0, 0);
		viewer.setBackgroundColor( 250.0, 250.0, 250.0 );
		unsigned long i = 0;
		while(!viewer.wasStopped ()) {
			viewer.addPointCloud(M, keypoints_color_handler, std::to_string(i));
			//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, std::to_string(i));
			viewer.spinOnce ();
			++i;
		}
/*
	pcl::visualization::CloudViewer viewer ("Cloud M viewer");
	while (!viewer.wasStopped()) {
	  viewer.showCloud (M);
	  boost::this_thread::sleep (boost::posix_time::milliseconds(2000));
	}
*/
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "registro");

	boost::thread ts(magia);

	// Tt = I
	Eigen::Matrix4f Tt;
	Tt << 
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	const size_t t = numero_de_nubes;

	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setLeafSize(0.025f, 0.025f, 0.025f);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filtroSOR;
	filtroSOR.setMeanK (100);
	filtroSOR.setStddevMulThresh (0.25);

	for ( auto i = 1; i < t; ++i ) {
		std::cout << RED << " Para i = " << i << ' ' << RESET << '\n';

		// Cargamos los datos
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (nubes + "X" + std::to_string( i ) + ".pcd", *cloud1);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (nubes + "X" + std::to_string( i+1 ) + ".pcd", *cloud2);
		
		puts("->NUBE inicial");
		std::cout << "Ci.size()   = " << cloud1->size() << "\n";
		std::cout << "Ci+1.size() = " << cloud2->size() << "\n";

		// APLICAMOS VOXEL GRID
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
		vGrid.setInputCloud(cloud1);
		vGrid.filter(*cloud_filtered1);
		vGrid.setInputCloud(cloud2);
		vGrid.filter(*cloud_filtered2);

		puts("->NUBE VG");
		std::cout << "Ci.size()   = " << cloud_filtered1->size() << "\n";
		std::cout << "Ci+1.size() = " << cloud_filtered2->size() << "\n";

		// APLICAMOS SOR
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeSOR1 (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeSOR2 (new pcl::PointCloud<pcl::PointXYZRGB>);
		filtroSOR.setInputCloud (cloud_filtered1);
		filtroSOR.filter (*nubeSOR1);
		filtroSOR.setInputCloud (cloud_filtered2);
		filtroSOR.filter (*nubeSOR2);

		puts("->NUBE SOR");
		std::cout << "Ci.size()   = " << nubeSOR1->size() << "\n";
		std::cout << "Ci+1.size() = " << nubeSOR2->size() << "\n";

		computeIteration( M, Tt, nubeSOR1, nubeSOR2 );
	}

	getchar();
	std::cout << "Tt\n" << Tt;

	return 0;
}
