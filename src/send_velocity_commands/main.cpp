#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

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

class RobotDriver {
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  ros::NodeHandle nh_stamp_;

  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_stamp_pub_;
  ros::Publisher cmd_pub_;

  // teclas
  char teclas_ [5];

public:
  //! ROS node initialization
  RobotDriver (ros::NodeHandle &nh) {
    nh_ = nh;

    //set up the publisher for the cmd_vel topic
    cmd_stamp_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("chatter", 1);
    cmd_pub_ = nh_stamp_.advertise<geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1);

    teclas_[0] = 'w'; // delante
    teclas_[1] = 'a'; // izquierda
    teclas_[2] = 'd'; // derecha
    teclas_[3] = 'p'; // parar bucle infinito
    teclas_[4] = 'r'; // obtener nube de puntos
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard() {
    std::cout << 
      "forward: " << teclas_[0] << 
      " left: "   << teclas_[1] <<
      " right: "  << teclas_[2] <<
      " exit: "   << teclas_[3] <<
      " capture: "<< teclas_[4] << "\n";

    //sending commands type
    geometry_msgs::TwistStamped base_stamp_cmd;  
    geometry_msgs::Twist base_cmd;

    char cmd[50];

    while(nh_.ok() && nh_stamp_.ok()) {
      std::cin.getline(cmd, 50);

      if (cmd[0] != teclas_[0] && cmd[0] != teclas_[1] && cmd[0] != teclas_[2] && cmd[0] != teclas_[3] && cmd[0] != teclas_[4]) {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }
      
      base_stamp_cmd.twist.linear.x = base_stamp_cmd.twist.linear.y = base_stamp_cmd.twist.angular.z = 0;
      base_stamp_cmd.header.stamp = ros::Time::now();
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
    
      //move forward
      if (cmd[0] == teclas_[0]) {
        base_stamp_cmd.twist.linear.x = base_cmd.linear.x = 0.25;
      } 
      //turn left (yaw) and drive forward at the same time
      else if (cmd[0] == teclas_[1]) {
        base_stamp_cmd.twist.angular.z = base_cmd.angular.z = 0.35;
      } 
      //turn right (yaw) and drive forward at the same time
      else if (cmd[0] == teclas_[2]) {
        base_stamp_cmd.twist.angular.z = base_cmd.angular.z = -0.35;
      } 
      //quit
      else if (cmd[0] == teclas_[3]) {
        break;
      }
      // capturar
      else if (cmd[0] == teclas_[4]) {
  	    ros::spinOnce();
      }

      //publish the assembled command
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

int capturas = 1;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

void simpleVis () {
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

  while (!viewer.wasStopped()) {
    viewer.showCloud (visu_pc);

    char *intStr = itoa(capturas);
    string numero = string(intStr);

    pcl::io::savePCDFile ("nube" + numero + ".pcd", visu_pc, true);

    capturas++;
    boost::this_thread::sleep (boost::posix_time::milliseconds(1000));
  }
}

void callback (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
/*
  cout << "Puntos capturados: " << nube->size() << endl;

  pcl::VoxelGrid<pcl::PointXYZRGB > voxelGrid;

  voxelGrid.setInputCloud (nube);
  voxelGrid.setLeafSize (0.05f, 0.05f, 0.05f);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeVG (new pcl::PointCloud<pcl::PointXYZRGB>);
  voxelGrid.filter (*nubeVG);

  cout << "Puntos tras VG: " << nubeVG->size() << endl;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filtroSOR;

  filtroSOR.setInputCloud (nubeVG);
  filtroSOR.setMeanK (100);
  filtroSOR.setStddevMulThresh (0.25);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nubeSOR (new pcl::PointCloud<pcl::PointXYZRGB>);
  filtroSOR.filter (*nubeSOR);

  cout << "Puntos tras SOR: " << nubeSOR->size() << endl;

  visu_pc = nubeSOR;
  */
  visu_pc = nube;
}

int main (int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  RobotDriver driver (nh);

  ros::NodeHandle nhCloud;
  ros::Subscriber imageSub = nhCloud.subscribe <pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth/points", 1, callback);
  boost::thread t (simpleVis);

  driver.driveKeyboard ();
}
