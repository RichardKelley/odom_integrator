#include "odom_integrator/odom_integrator.h"

#include <ros/ros.h>
#include "gnss_driver/Gps.h"

#include <yaml-cpp/yaml.h>
#include <std_msgs/Float64.h>

#include <cmath>

namespace odom_integrator {

  OdomIntegratorNode::OdomIntegratorNode(const std::string& filename)
    : conf_filename_{filename} {

    gnss_odom_sub_ = nh_.subscribe<gnss_driver::Gps>("/gnss_driver/odometry", 1,
						     &OdomIntegratorNode::gnss_callback,
						     this);

    distance_traveled_pub_ = nh_.advertise<std_msgs::Float64>("/odom_integrator/distance", 10);
    
    distance_traveled_timer_ = nh_.createTimer(ros::Duration(0.1),
					       &OdomIntegratorNode::timer_callback,
					       this);

    load_configuration(filename);
    
  }

  void OdomIntegratorNode::gnss_callback(const gnss_driver::GpsConstPtr& gps_msg) {

    if (first_) {
      last_odom_ = *gps_msg;
      first_ = false;
      return;
    }

    if (use_position_) {
    
      auto last_x = last_odom_.localization.position.x;
      auto last_y = last_odom_.localization.position.y;
      auto last_z = last_odom_.localization.position.z;
      
      auto x = gps_msg->localization.position.x;
      auto y = gps_msg->localization.position.y;
      auto z = gps_msg->localization.position.z;
      
      // compute the distance from the last point
      auto delta_x = std::sqrt((last_x - x)*(last_x - x) +
			       (last_y - y)*(last_y - y) +
			       (last_z - z)*(last_z - z));
      
      distance_traveled_ += delta_x;
    } else { // use velocity

      auto dt = gps_msg->header.stamp.toSec() - last_odom_.header.stamp.toSec();
      
      auto v_x = gps_msg->localization.linear_velocity.x;
      auto v_y = gps_msg->localization.linear_velocity.y;
      auto v_z = gps_msg->localization.linear_velocity.z;
      
      auto delta_x = dt * std::sqrt(v_x * v_x + v_y * v_y + v_z * v_z);

      distance_traveled_ += delta_x;
      
    }
    
    // update the last point
    last_odom_ = *gps_msg;
    
  }

  void OdomIntegratorNode::timer_callback(const ros::TimerEvent& event) {
    std_msgs::Float64 distance;
    distance.data = distance_traveled_;
    distance_traveled_pub_.publish(distance);

  }
  
  bool OdomIntegratorNode::load_configuration(std::string filename) {
    std::cerr << "Loading configuration: " << filename << std::endl;

    YAML::Node c = YAML::LoadFile(filename);

    if (c.Type() != YAML::NodeType::Map) {
      ROS_ERROR("Invalid configuration.");
      return false;
    }

    if (c["odom_type"].as<std::string>() == "gps") {
      ROS_INFO("Using position-based odometry.");
      use_position_ = true;
    } else if (c["odom_type"].as<std::string>() == "vel") {
      ROS_INFO("Using velocity-based odometry.");
      use_position_ = false;
    } else {
      ROS_ERROR("Unknown odometry type requested.");
      return false;
    }
    
  }

  
  
}
