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

void filtroVoxelGrid (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nube, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeVG) {
  pcl::VoxelGrid <pcl::PointXYZRGB> voxelGrid;

  voxelGrid.setInputCloud (nube);
  voxelGrid.setLeafSize (0.025f, 0.025f, 0.025f);
  voxelGrid.filter (*nubeVG);
}

void filtroSor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeVG, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeSOR) {
  // Probar parametros http://wiki.ros.org/pcl_ros/Tutorials/filters
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filtroSOR;

  filtroSOR.setInputCloud (nubeVG);
  filtroSOR.setMeanK (100);
  filtroSOR.setStddevMulThresh (0.25);
  filtroSOR.filter (*nubeSOR);
}

void estimacionNormales(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeSOR, pcl::PointCloud<pcl::PointNormal>::Ptr& nubeNormales) {
  // http://docs.ros.org/indigo/api/pcl_ros/html/classpcl__ros_1_1NormalEstimation.html
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> estimacionNormal;

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1search_1_1KdTree.html
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr arbolNormales(new pcl::search::KdTree<pcl::PointXYZRGB>());

  estimacionNormal.setInputCloud(nubeSOR);
  estimacionNormal.setSearchMethod(arbolNormales);
  estimacionNormal.setRadiusSearch(0.05);
  estimacionNormal.compute(*nubeNormales);
}

void extraccionPuntosCaracteristicos(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeSOR, pcl::PointCloud<pcl::PointNormal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointWithScale>::Ptr& resultado) {
  for(size_t i = 0; i < nubeNormales->points.size(); ++i) {
    nubeNormales->points[i].x = nubeSOR->points[i].x;
    nubeNormales->points[i].y = nubeSOR->points[i].y;
    nubeNormales->points[i].z = nubeSOR->points[i].z;
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
  sift.setInputCloud (nubeNormales);
  sift.compute (*resultado);
}

void extraccionCaracteristicas(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeSOR, pcl::PointCloud<pcl::PointNormal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointWithScale>::Ptr& puntosClave, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& resultado) {
  // http://docs.ros.org/indigo/api/pcl_ros/html/classpcl__ros_1_1FPFHEstimation.html
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;

  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1search_1_1KdTree.html
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr arbol(new pcl::search::KdTree<pcl::PointXYZRGB> ());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr puntosClaveAux(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::copyPointCloud (*puntosClave, *puntosClaveAux);

  fpfh.setInputCloud (puntosClaveAux);
  fpfh.setInputNormals (nubeNormales);
  fpfh.setSearchSurface (nubeSOR);
  fpfh.setSearchMethod (arbol);
  fpfh.setRadiusSearch (0.07);
  fpfh.compute (*resultado);
}

void obtencionCorrespondencias(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& origen, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& destino, pcl::Correspondences& resultado) {
  // http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1registration_1_1CorrespondenceEstimation.html
  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimacion;

  estimacion.setInputSource (origen);
  estimacion.setInputTarget (destino);
  estimacion.determineReciprocalCorrespondences (resultado);
  // Probar distancias como segundo parámetro 0.07
}

Eigen::Matrix4f rechazarCorrespondencias(pcl::PointCloud<pcl::PointWithScale>::Ptr& caracteristicasOrigen, pcl::PointCloud<pcl::PointWithScale>::Ptr& caracteristicasDestino, pcl::Correspondences& correspondencias, pcl::Correspondences& correspondenciasRechazadas) {
  // http://wiki.ros.org/eigen_conversions
  pcl::CorrespondencesConstPtr correspondencia (new pcl::Correspondences(correspondencias));
  // http://docs.ros.org/diamondback/api/pcl/html/classpcl_1_1registration_1_1CorrespondenceRejectorSampleConsensus.html
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> muestra;

  muestra.setInputSource (caracteristicasOrigen);
  muestra.setInputTarget (caracteristicasDestino);
  muestra.setInlierThreshold (0.025);
  muestra.setMaximumIterations (10000);
  muestra.setRefineModel (true);
  muestra.setInputCorrespondences (correspondencia);
  muestra.getCorrespondences (correspondenciasRechazadas);

  return muestra.getBestTransformation();
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeVG (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeSOR (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr nubeNormales (new pcl::PointCloud<pcl::PointNormal>);

  if (capturaInicial) {
    filtroVoxelGrid (nube, nubeVG);
    filtroSor (nubeVG, nubeSOR);
    estimacionNormales (nubeSOR, nubeNormales);
    extraccionPuntosCaracteristicos (nubeSOR, nubeNormales, puntosCaracteristicos);
    extraccionCaracteristicas (nubeSOR, nubeNormales, puntosCaracteristicos, fpfhs);

    capturaInicial = false;
    visu_pc = nubeSOR;
  }
  else {
    visu_pcAnterior = visu_pc;
    puntosCaracteristicosAnterior = puntosCaracteristicos;
    fpfhsAnterior = fpfhs;

    filtroVoxelGrid (nube, nubeVG);
    filtroSor (nubeVG, nubeSOR);
    estimacionNormales (nubeSOR, nubeNormales);

    pcl::PointCloud<pcl::PointWithScale>::Ptr puntosCaracteristicosSiguiente (new pcl::PointCloud<pcl::PointWithScale> ());
    extraccionPuntosCaracteristicos (nubeSOR, nubeNormales, puntosCaracteristicosSiguiente);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhsSiguiente (new pcl::PointCloud<pcl::FPFHSignature33> ());
    extraccionCaracteristicas (nubeSOR, nubeNormales, puntosCaracteristicosSiguiente, fpfhsSiguiente);

    pcl::Correspondences correspondencias;
    obtencionCorrespondencias(fpfhs, fpfhsSiguiente, correspondencias);

    pcl::Correspondences correspondenciasRechazadas;
    Eigen::Matrix4f matrizTransformacion = rechazarCorrespondencias(puntosCaracteristicos, puntosCaracteristicosSiguiente, correspondencias, correspondenciasRechazadas);

    if (correspondenciasRechazadas.size() > (correspondencias.size() / 1.25)) {
      pcl::PointCloud<pcl::PointNormal>::Ptr nubeNormalesAux (new pcl::PointCloud<pcl::PointNormal>);
      estimacionNormales(visu_pc, nubeNormalesAux);

      pcl::PointCloud<pcl::PointWithScale>::Ptr puntosCaracteristicosAux (new pcl::PointCloud<pcl::PointWithScale> ());
      extraccionPuntosCaracteristicos(visu_pc, nubeNormalesAux, puntosCaracteristicosAux);

      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhsAux (new pcl::PointCloud<pcl::FPFHSignature33> ());
      extraccionCaracteristicas(visu_pc, nubeNormalesAux, puntosCaracteristicosAux, fpfhsAux);

      pcl::Correspondences correspondenciasAux;
      obtencionCorrespondencias(fpfhsAux, fpfhsSiguiente, correspondenciasAux);

      pcl::Correspondences correspondenciasRechazadasAux;
      Eigen::Matrix4f matrizTransformacionAux = rechazarCorrespondencias (puntosCaracteristicosAux, puntosCaracteristicosSiguiente, correspondenciasAux, correspondenciasRechazadasAux);

      if (correspondenciasRechazadas.size() > (correspondencias.size() / 1.25)) {
        visu_pc = visu_pcAnterior;
        puntosCaracteristicos = puntosCaracteristicosAnterior;
        fpfhs = fpfhsAnterior;

        return;
      }

      puntosCaracteristicos = puntosCaracteristicosSiguiente;
      fpfhs =  fpfhsSiguiente;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeTransformada (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*visu_pc, *nubeTransformada, matrizTransformacionAux);

      *visu_pc = *nubeTransformada + *nubeVG;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeVGAux (new pcl::PointCloud<pcl::PointXYZRGB>);
      filtroVoxelGrid (visu_pc, nubeVGAux);

      visu_pc = nubeVGAux;

      return;
    }

    puntosCaracteristicos = puntosCaracteristicosSiguiente;
    fpfhs = fpfhsSiguiente;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeTransformadaSiguiente (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*visu_pc, *nubeTransformadaSiguiente, matrizTransformacion);

    *visu_pc = *nubeTransformadaSiguiente + *nubeVG;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeVGSiguiente (new pcl::PointCloud<pcl::PointXYZRGB>);
    filtroVoxelGrid (visu_pc, nubeVGSiguiente);

    visu_pc = nubeVGSiguiente;
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
