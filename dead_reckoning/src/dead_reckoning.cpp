// ROS Includes
#include <tf/transform_listener.h>

#include <tf/tf.h>

// C++ Includes
#include <math.h>

// Node Includes
#include "dead_reckoning.h"

DeadReckoning::DeadReckoning()
  {
  }

DeadReckoning::~DeadReckoning()
{
}

bool DeadReckoning::init()
{

  _vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    tf::TransformListener listener;
    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(4.0));

}
bool DeadReckoning::drive(float distance, float speed)
{
   bool forward = (distance >= 0);
   
		
   // Record Start
   tf::StampedTransform transform;
   _transform_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
   
   tf::Vector3 startTranslation = transform.getOrigin();
   tf::Quaternion startRotation = transform.getRotation();
   bool done = false;
   
   geometry_msgs::Twist vel_msg;
   vel_msg.linear.x = forward == true ? speed : -speed; // Forward/Reverse Speed in m/s
   vel_msg.angular.z = 0; // No angular
   while(ros::ok())
   {
     try
     {
      _transform_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      tf::Vector3 currentTranslation = transform.getOrigin();
      tf::Quaternion currentRotation = transform.getRotation();
      
      float dx = currentTranslation[0] - startTranslation[0];
      float dy = currentTranslation[1] - startTranslation[1];
      
      float distanceMoved = sqrt(dx * dx + dy * dy);
      
      ROS_INFO("Distance Moved: %f", distanceMoved);
      bool arrived = forward == true ? distanceMoved >= distance : distanceMoved >= -distance; // Check Distance Travelled.  Need Tolerance HEre?
      
      if(arrived)
      {
	break;
      }
      else
      {
	ROS_INFO("Sending Vel Command: %f", vel_msg.linear.x);
	_vel_pub.publish(vel_msg); // Publish Velocity
      }
      
  
     }
     catch (tf::TransformException ex)
     {
       ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
     } 
     ros::Duration(0.1).sleep();
   }
   
  // Stop
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  _vel_pub.publish(vel_msg);
                
  return done;

}
bool DeadReckoning::turn(float angle, float angularSpeed)
{
  bool ccw = (angle >= 0); // counter clockwise rotation
	tfScalar roll, pitch, yaw;	
  // Record Start
   tf::StampedTransform transform;
   _transform_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
   
   tf::Quaternion startRotation = transform.getRotation();
   
   tf::Matrix3x3(startRotation).getRPY(roll, pitch, yaw);
   
   //float startAngle = 2 * atan2(startRotation[2], startRotation[3]);
float startAngle = yaw;
   ROS_INFO( "Start Angle: %f", startAngle);
   float previousAngle = startAngle;
   float angleOffset = 0.0;
                
   bool done = false;

    bool arrived = false;
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = 0.0; //# going forward m/s
  if (ccw)
    vel_msg.angular.z = angularSpeed;
  else
    vel_msg.angular.z = -angularSpeed;
		
  vel_msg.angular.z = ccw ? angularSpeed : -angularSpeed;
  while(ros::ok())
   {
     try
     {
      _transform_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      tf::Quaternion currentRotation = transform.getRotation();
      
      tf::Matrix3x3(currentRotation).getRPY(roll, pitch, yaw);
      float currentAngle = yaw;
      //float currentAngle = 2 * atan2(currentRotation[2], currentRotation[3]);
     // ROS_INFO("Current Angle: %f", currentAngle * 180 / M_PI);
//ROS_INFO("Current Angle Quat: %f", currentRotation.z());

      //# we need to handle roll over of the angle
//       if (currentAngle * previousAngle < 0 && fabs(currentAngle - previousAngle) > M_PI / 2.0){
// 	if (currentAngle > previousAngle){
// 	//  ROS_INFO("subtracting");
//           angleOffset = angleOffset - 2 * M_PI;
// 	}
//         else{
// 	  //ROS_INFO("adding");
//           angleOffset = angleOffset + 2 * M_PI;
// 	  }
//       }
      
      if(currentAngle * previousAngle < 0 && currentAngle < 0.0)
      {
	
	//ROS_INFO("Offset 1: ");
	//currentAngle = (2 * M_PI) + currentAngle;
	angleOffset = angleOffset + (2 * M_PI);
      }
      else if(currentAngle * previousAngle < 0 &&  currentAngle > 0.0)
      {
	//ROS_INFO("Offset 2: ");
	//currentAngle = currentAngle - (2 * M_PI);
	angleOffset = angleOffset + (2 * M_PI);
      }
      previousAngle = currentAngle;
      
      float angleTurned = currentAngle + angleOffset - startAngle;
      
                                
      ROS_INFO("Current Angle New: %f", currentAngle * 180 / M_PI);
      ROS_INFO("Angle Turned: %f", angleTurned);

      if (ccw)
	arrived = (angleTurned >= angle);
      else
	arrived = (angleTurned <= angle);
                                
      //ROS_INFO("Arrived: %d", arrived);
      if(arrived)
      {
	break;
      }
      else
      {
	//ROS_INFO("Sending Vel Command: %f", vel_msg.linear.x);
	_vel_pub.publish(vel_msg); // Publish Velocity
      }
		  
	      
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    } 
    
    ros::Duration(0.1).sleep();
}

  // Stop
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  _vel_pub.publish(vel_msg);
  return done;
}

 // Main Startup Function
int main(int argc, char** argv)
 {
  
  ros::init(argc, argv, "dead_reckoning");
  //ros::Rate rate(50.0);
    // Create Base Controller;
  DeadReckoning dead_reckoning;
  dead_reckoning.init();
  dead_reckoning.drive(5.0, 0.5);
  //dead_reckoning.turn(180, 0.5);
  //dead_reckoning.drive(5.0, 0.5);
  
  
  
  
  //ros::Rate rate(10.0);
  //geometry_msgs::Twist vel_msg;
  
  //ros::spin();
}
