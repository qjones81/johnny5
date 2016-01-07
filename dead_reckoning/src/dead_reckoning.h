#ifndef DEAD_RECKONING_CORE_H
#define DEAD_RECKONING_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using std::string;

class DeadReckoning
{
public:
  //! Constructor.
  DeadReckoning();

  //! Destructor.
  ~DeadReckoning();

  bool init();

  bool drive(float distance, float speed);
  bool turn(float angle, float angularSpeed);
  
private:
  
// Node NodeHandle
  ros::NodeHandle _nh;
  
  // Publishers
  ros::Publisher _vel_pub;

  // Subscribers
  ros::Subscriber _cmd_vel_sub;
  
  // Transform Listener
  tf::TransformListener _transform_listener;
  
};

#endif // BASE_CONTROLLER_CORE_H
