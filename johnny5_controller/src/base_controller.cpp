// ROS Includes
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// C++ Includes
#include <math.h>

// Node Includes
#include "base_controller.h"
  
BaseController::BaseController() : 
  _base_frame("base_link"), // For TF
  _odom_frame("odom"), // For TF
  _base_controller_rate(10.0), // 10 hz
  _base_controller_timeout(1.0), // 1 Second
  _stopped(true),
  _wheel_diameter(0.1524), // 6 in
  _wheel_track(0.381),	  // 15 in
  _encoder_resolution(64), 
  _gear_reduction(29),
  _accel_limit(0.5),
  _ticks_per_meter(3876.53),
  _max_accel(1.0),
  _x(0),
  _y(0),
  _th(0),
  _encoder_left(0),
  _encoder_right(0),
  _v_left(0),
  _v_right(0),
  _v_target_left(0),
  _v_target_right(0),
  _last_cmd_vel(0)
  {

  }

BaseController::~BaseController()
{
}
void BaseController::spin()
{
  _current_time = ros::Time::now();
  _last_time = _current_time;
    ros::Rate rate(_base_controller_rate);
    while(ros::ok())
    {
        ros::spinOnce();
	_current_time = ros::Time::now();
	_last_time = _current_time;
	update();
        rate.sleep();
    }
}
bool BaseController::init()
{
  // Encoder Ticks per Meter of Travel
  _ticks_per_meter = _encoder_resolution * _gear_reduction / (_wheel_diameter * M_PI);

  // Max Acceleration
  _max_accel = _accel_limit * _ticks_per_meter / _base_controller_rate;
  
  // Time Variables
  _current_time = ros::Time::now();
  _last_time = ros::Time::now();
  
  // Microcontroller Initializations/Connect
  _microcontroller.init();
  _microcontroller.connect();
  
  // Reset Encoders back to zero
  _microcontroller.reset_encoders();
  
  // Update Publishers and Subscribers
  _odom_pub = _nh.advertise<nav_msgs::Odometry>("/odom", 50);
  _cmd_vel_sub = _nh.subscribe("/cmd_vel", 1, &BaseController::cmd_vel_callback, this);
}
void BaseController::update()
{
  _current_time = ros::Time::now();
  
  // Get Encoder Values
  int left_enc;
  int right_enc;
  bool bSuccess = _microcontroller.get_encoder_ticks(left_enc, right_enc);
  ROS_INFO("Left: %d, Right: %d", left_enc, right_enc);
  //try
  //catch
  // blah
  
  
  // Update Time Values
  _dt = _current_time - _last_time;
  _last_time = _current_time;
  double dt = _dt.toSec();
  
  // Compute Odometry
  
  _microcontroller.drive(_v_target_left, _v_target_right);
}
void BaseController::stop()
{
  // Set Motor Speeds to Zero/Off
  //micro.drive(0,0);
  _stopped = true;
  
}


void BaseController::cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
  _last_cmd_vel = ros::Time::now();
  float x = vel_cmd.linear.x;
  float th = vel_cmd.angular.z;
  float right = 00;
  float left = 0;
  if(x == 0) // Turning in place (This is wrong)
  {
    right = th * _wheel_track * _gear_reduction * 0.5;
    left = -right;
  }
  else if(th == 0) // Pure Forward/Backwards Motion
  {
    left = right = x;
  }
  else// Turn Rotate. Reevaluate this
  {
    left = x - th * _wheel_track * _gear_reduction * 0.5;
    right = x + th * _wheel_track * _gear_reduction * 0.5;
  }
  _v_target_left = (int)(left * _ticks_per_meter);
  _v_target_right = (int)(right * _ticks_per_meter);
}
 
 // Main Startup Function
int main(int argc, char** argv)
 {
  
  ros::init(argc, argv, "base_controller");
  // Create Base Controller;
  BaseController base_controller;
  base_controller.init();
  base_controller.spin();
}
