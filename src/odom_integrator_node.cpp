#include <iostream>
#include <string>

/*
  The next two headers are ROS-specific. The first is necessary to use
  ROS at all - you will probably include it in everything ROS-related
  that you use.

  The package.h header is useful for working with paths in ROS. We're
  going to use it to get the path to a configuration file.
*/
#include <ros/ros.h>
#include <ros/package.h>

#include "odom_integrator/odom_integrator.h"

int main(int argc, char **argv) {

  // Start ROS.
  ros::init(argc, argv, "odom_integrator_node");

  /*
    Get the path of the odom_integrator package
   */
  std::string path = ros::package::getPath("odom_integrator");
  std::cerr << "Found package path: " << path << std::endl;

  std::string full_path = path + "/conf/conf.yaml";
  
  std::cerr << "Using full config path: " << full_path << std::endl;
  
  // Create object representing the node.
  odom_integrator::OdomIntegratorNode node{full_path};
  
  ros::spin();
  return 0;
}

