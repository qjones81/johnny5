#ifndef BASE_CONTROLLER_CORE_H
#define BASE_CONTROLLER_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "microcontroller_bridge.h"

using std::string;

class BaseController
{
public:
  //! Constructor.
  BaseController();

  //! Destructor.
  ~BaseController();

  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
  bool init();
  void update();
  void spin();
  void stop();
private:
  
  std::string _base_frame;
  std::string _odom_frame;

  float _base_controller_rate;
  float _base_controller_timeout;

  bool _stopped;

  // Platform Options
  float _wheel_diameter;
  float _wheel_track;
  float _encoder_resolution;
  float _gear_reduction;
  float _accel_limit;
  float _ticks_per_meter;
  float _max_accel;

  // Position Data -> To Odometry Class
  float _x;
  float _y;
  float _th;

  // Encoder Data
  int _encoder_left;
  int _encoder_right;
  
  // Speed Data (PPS aka ints)
  int _v_left;
  int _v_right;
  int _v_target_left;
  int _v_target_right;

  ros::Time _last_cmd_vel;

  
  ros::Time _current_time, _last_time;
  ros::Duration _dt;
  MicroControllerBridge _microcontroller;

  // Node NodeHandle
  ros::NodeHandle _nh;
  
  // Publishers
  ros::Publisher _odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  
  // Subscribers
  ros::Subscriber _cmd_vel_sub;

};

#endif // BASE_CONTROLLER_CORE_H
