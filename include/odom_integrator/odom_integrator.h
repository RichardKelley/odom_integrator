#ifndef ODOM_INTEGRATOR_H
#define ODOM_INTEGRATOR_H

#include <string>

#include <ros/ros.h>

#include <gnss_driver/Gps.h>

/*
  Why the namespace? In larger projects, you may share code from
  multiple packages, and there's a good chance you'll hit naming
  conflicts. If you have another strategy for preventing or resolving
  such conflicts, then you don't necessarily have to use namespaces
  like this. But if you don't have such a strategy, I'd recommend this
  approach.
*/
namespace odom_integrator {

  /*
    The OdomIntegratorNode receives odometry information from the GNSS
    system, and integrates that over time to get an estimate of the
    distance traveled by the car. 

    To demonstrate the use of timers, it publishes its estimate of the
    total distance traveled 10 times per second.
    
    The concept here is that we should wrap the functionality of our
    program in a class. Instantiating this class acquires the
    resources our system needs to run, and when the program is over
    the class destructor cleans up for us (standard RAII).

    We use the convention that member variables use a trailing
    underscore.
  */
  class OdomIntegratorNode {
  public:

    OdomIntegratorNode(const std::string& filename);

    void gnss_callback(const gnss_driver::GpsConstPtr& gps_msg); 
    void timer_callback(const ros::TimerEvent& event);

    
  private:

    bool load_configuration(std::string filename);
    
    ros::NodeHandle nh_;

    ros::Subscriber gnss_odom_sub_;
    ros::Publisher distance_traveled_pub_;
    ros::Timer distance_traveled_timer_;

    std::string conf_filename_;

    gnss_driver::Gps last_odom_{};

    bool use_position_ = false;
    bool first_ = true;    
    double distance_traveled_ = 0.0;
    
    
  };

}

#endif // ODOM_INTEGRATOR_H
