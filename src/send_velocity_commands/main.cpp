#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

class RobotDriver {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_stamp_;

  ros::Publisher cmd_stamp_pub_;
  ros::Publisher cmd_pub_;

  char teclas_ [5];

public:
  RobotDriver (ros::NodeHandle &nh) {
    nh_ = nh;

    cmd_stamp_pub_ = nh_.advertise <geometry_msgs::TwistStamped> ("chatter", 1);
    cmd_pub_ = nh_stamp_.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1);

    teclas_[0] = 'w'; // delante
    teclas_[1] = 'a'; // izquierda
    teclas_[2] = 'd'; // derecha
    teclas_[3] = 'p'; // parar bucle infinito
    teclas_[4] = 'r'; // parar bucle infinito
  }

  bool driveKeyboard() {
    std::cout << 
      "forward: " << teclas_[0] << 
      " left: "   << teclas_[1] <<
      " right: "  << teclas_[2] <<
      " exit: "   << teclas_[3] <<
      " capture: "<< teclas_[4] << "\n";

    geometry_msgs::TwistStamped base_stamp_cmd;  
    geometry_msgs::Twist base_cmd;

    char cmd[50];

    while (nh_.ok() && nh_stamp_.ok()) {
      std::cin.getline (cmd, 50);
      if (cmd[0] != teclas_[0] && cmd[0] != teclas_[1] && cmd[0] != teclas_[2] && cmd[0] != teclas_[3] && cmd[0] != teclas_[4]) {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      base_stamp_cmd.twist.linear.x = base_stamp_cmd.twist.linear.y = base_stamp_cmd.twist.angular.z = 0;
      base_stamp_cmd.header.stamp = ros::Time::now();
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
    
      // delante
      if (cmd[0] == teclas_[0]) {
        base_stamp_cmd.twist.linear.x = base_cmd.linear.x = 0.25;
      } 
      // izquierda
      else if (cmd[0] == teclas_[1]) {
        base_stamp_cmd.twist.angular.z = base_cmd.angular.z = 0.5;
      } 
      // derecha
      else if (cmd[0] == teclas_[2]) {
        base_stamp_cmd.twist.angular.z = base_cmd.angular.z = -0.5;
      } 
      // terminar
      else if (cmd[0] == teclas_[3]) {
        break;
      }
      // capturar
      else if (cmd[0] == teclas_[4]) {
  	    ros::spinOnce();
      }

      std::cout << 
        "x: " << base_stamp_cmd.twist.linear.x  << 
        " y: " << base_stamp_cmd.twist.linear.y  <<
        " z: " << base_stamp_cmd.twist.angular.z <<
        " stamp: " << base_stamp_cmd.header.stamp << "\n";

      cmd_stamp_pub_.publish (base_stamp_cmd);
      cmd_pub_.publish (base_cmd);
    }

    return true;
  }
};

bool capturaInicial = true;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointWithScale>::Ptr puntosCaracteristicos (new pcl::PointCloud<pcl::PointWithScale> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pcAnterior (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointWithScale>::Ptr puntosCaracteristicosAnterior (new pcl::PointCloud<pcl::PointWithScale> ());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhsAnterior (new pcl::PointCloud<pcl::FPFHSignature33> ());

void simpleVis () {
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while (!viewer.wasStopped()) {
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep (boost::posix_time::milliseconds(1000));
	}
}

void filtroVoxelGrid (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube_VG) {
  pcl::VoxelGrid <pcl::PointXYZRGB> vGrid;

  vGrid.setInputCloud (nube);
  vGrid.setLeafSize (0.025f, 0.025f, 0.025f);
  vGrid.filter (*nube_VG);
}

void filtroSor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube_VG, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube_SOR) {
  // Probar parametros http://wiki.ros.org/pcl_ros/Tutorials/filters
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filtro_SOR;

  filtro_SOR.setInputCloud (nube_VG);
  filtro_SOR.setMeanK (100);
  filtro_SOR.setStddevMulThresh (0.25);
  filtro_SOR.filter (*nube_SOR);
}

void estimacionNormales(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube_SOR, pcl::PointCloud<pcl::PointNormal>::Ptr& nube_normales) {
  // http://docs.ros.org/indigo/api/pcl_ros/html/classpcl__ros_1_1NormalEstimation.html
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> estimacionNormal;

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1search_1_1KdTree.html
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr arbol_normales(new pcl::search::KdTree<pcl::PointXYZRGB>());

  estimacionNormal.setInputCloud(nube_SOR);
  estimacionNormal.setSearchMethod(arbol_normales);
  estimacionNormal.setRadiusSearch(0.05);
  estimacionNormal.compute(*nube_normales);
}

void extraccionPuntosCaracteristicos(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube_SOR, pcl::PointCloud<pcl::PointNormal>::Ptr& nube_normales) {
  for(size_t i = 0; i < nube_normales->points.size(); ++i) {
    nube_normales->points[i].x = nube_SOR->points[i].x;
    nube_normales->points[i].y = nube_SOR->points[i].y;
    nube_normales->points[i].z = nube_SOR->points[i].z;
  }

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1SIFTKeypoint.html
  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;

  const static float escalaMinima = 0.01f;
  const static int numeroOctavas = 3;
  const static int escalasPorOctavas = 4;
  const static float contrasteMinimmo = 0.001f;

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1search_1_1KdTree.html
  pcl::search::KdTree<pcl::PointNormal>::Ptr arbol (new pcl::search::KdTree<pcl::PointNormal> ());

  sift.setSearchMethod (arbol);
  sift.setScales (escalaMinima, numeroOctavas, escalasPorOctavas);
  sift.setMinimumContrast (contrasteMinimmo);
  sift.setInputCloud (nube_normales);
  sift.compute (*puntosCaracteristicos);
}

void extraccionCaracteristicas(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals) {
  // http://docs.ros.org/indigo/api/pcl_ros/html/classpcl__ros_1_1FPFHEstimation.html
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1search_1_1KdTree.html
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud (*puntosCaracteristicos, *keypoints_xyzrgb);

  fpfh.setInputCloud (keypoints_xyzrgb);
  fpfh.setInputNormals (cloud_normals);
  fpfh.setSearchSurface (cloud);
  fpfh.setSearchMethod (tree);
  fpfh.setRadiusSearch (0.07);
  fpfh.compute (*fpfhs);
}

void obtencionCorrespondencias(pcl::Correspondences& correspondences) {
  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1registration_1_1CorrespondenceEstimation.html
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;

  est.setInputSource (fpfhsAnterior);
  est.setInputTarget (fpfhs);
  est.determineReciprocalCorrespondences (correspondences);
  // Probar distancias como segundo parámetro 0.07 
}

Eigen::Matrix4f rechazarCorrespondencias(pcl::Correspondences& correspondencias, pcl::Correspondences& correspondenciasRechazadas) {
  // http://wiki.ros.org/eigen_conversions
  pcl::CorrespondencesConstPtr correspondencia (new pcl::Correspondences(correspondencias));
  // http://docs.ros.org/diamondback/api/pcl/html/classpcl_1_1registration_1_1CorrespondenceRejectorSampleConsensus.html
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> sac;

  sac.setInputSource (puntosCaracteristicosAnterior);
  sac.setInputTarget (puntosCaracteristicos);
  sac.setInlierThreshold (0.025);
  sac.setMaximumIterations (10000);
  sac.setRefineModel (true);
  sac.setInputCorrespondences (correspondencia);
  sac.getCorrespondences (correspondenciasRechazadas);

  return sac.getBestTransformation();
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_VG (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_SOR (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr nube_normales (new pcl::PointCloud<pcl::PointNormal>);

  if (capturaInicial) {
    filtroVoxelGrid (nube, nube_VG);
    filtroSor (nube_VG, nube_SOR);
    estimacionNormales (nube_SOR, nube_normales);
    extraccionPuntosCaracteristicos (nube_SOR, nube_normales);
    extraccionCaracteristicas (nube_SOR, nube_normales);

    capturaInicial = false;
    visu_pc = nube_SOR;
  }
  else {
    visu_pcAnterior = visu_pc;
    puntosCaracteristicosAnterior = puntosCaracteristicos;
    fpfhsAnterior = fpfhs;

    filtroVoxelGrid (nube, nube_VG);
    filtroSor (nube_VG, nube_SOR);
    estimacionNormales (nube_SOR, nube_normales);
    extraccionPuntosCaracteristicos (nube_SOR, nube_normales);
    extraccionCaracteristicas (nube_SOR, nube_normales);

    pcl::Correspondences correspondencias;
    pcl::Correspondences correspondenciasRechazadas;

    obtencionCorrespondencias(correspondencias);

    Eigen::Matrix4f matrizTransformacion = rechazarCorrespondencias(correspondencias, correspondenciasRechazadas);



    // TODO ELIMINAR
    visu_pc = nube_SOR;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver(nh);

  ros::NodeHandle nhCloud;
  ros::Subscriber imageSub = nhCloud.subscribe <pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth/points", 1, callback);
  boost::thread t(simpleVis);

  driver.driveKeyboard();
}
