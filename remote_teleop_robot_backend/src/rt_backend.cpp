/* File: rt_backend.cpp
 * Author: Anna Wong
 * Purpose: 
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include "rt_backend.h"

/*-----------------------------------------------------------------------------------*/
// Define variables here
#define THRESHOLD 0.8

/*-----------------------------------------------------------------------------------*/

// CONSTRUCTOR: this will get called whenever an instance of this class is created
RemoteTeleopClass::RemoteTeleopClass(ros::NodeHandle* nodehandle):nh_(*nodehandle) {

  ROS_INFO("in class constructor of RemoteTeleopClass");
  
  // Initialize the messy stuff
  initializeSubscribers();
  initializePublishers();
  initializeActions();
  
  // Initialize the internal variables
  angle_ = 0.0;
  turn_left_ = true;
  lin_vel_ = 0.0;
  ang_vel_ = 0.0;
  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;

}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::initializeSubscribers() {
  
  ROS_INFO("Initializing Subscribers");
  
  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("odom", 1, &RemoteTeleopClass::odom_callback, this);

}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::initializePublishers() {

  ROS_INFO("Initializing Publishers");
  
  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::initializeActions() {
  
  // Initialize the turn in place action server and start it
  turn_in_place_server_(nh_, "turn_in_place", boost::bind(&RemoteTeleopClass::turn_in_place_callback, this, _1), false);
  
  turn_in_place_server_.start();
  
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoal& goal) {
  
  // TODO: gray out rviz plugin buttons when turn is being executed
  
  // Get inputs from Rviz and store them in variables
  angle_ = goal.degrees;
  turn_left_ = goal.turn_left;
  
  // Convert from degrees to radians
  angle_ = angle_ * M_PI / 180;
  
  // TODO: update vel vars here, OR move this somewhere else
  lin_vel_ = 0.0;
  ang_vel_ = 0.5;
  
  // Tell robot to turn the desired angle
  turn_in_place();
  
  // Update the turn in place result and success fields
  turn_in_place_result_.success = true;
  turn_in_place_server_.setSucceeded(turn_in_place_result_);
  
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::odom_callback(const nav_msgs::Odometry& msg) {
  
  // TODO: this was really difficult to convert from python to c++, so I hope this works
  
  // Grab the odometry quaternion values out of the message
  tf::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);
  
  // Turn the quaternion values into a matrix
  tf::Matrix3x3 m(q);
  
  // Extract the euler angles from the matrix
  m.getRPY(roll_, pitch_, yaw_);
  
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopClass::turn_in_place() {
  
  // Create message to be sent
  geometry_msgs::Twist command;
  
  float goal_yaw = 0.0;
    
  if(turn_left_ == false) {
    // TURNING RIGHT
    goal_yaw = yaw_ - angle_;
    // Make sure the goal angle is within a valid range
    if(goal_yaw < -M_PI) {
      goal_yaw += 2*M_PI;
    }
  } else {
    // TURNING LEFT
    goal_yaw = yaw_ + angle_;
    // Make sure the goal angle is within a valid range
    if(goal_yaw > M_PI) {
      goal_yaw -= 2*M_PI;
    }
  }
  
  // Turn the robot until it reaches the desired angle
  while(abs(goal_yaw - yaw_) > THRESHOLD) {
    
    // Set the turn rate
    command.angular.z = ang_vel_ * (goal_yaw - yaw_);
    
    // Ensure the robot will be turning in the correct direction
    if(turn_left_ == true && command.angular.z < 0.0) {
      command.angular.z *= -1;
    } else if(turn_left_ == false && command.angular.z > 0.0) {
      command.angular.z *= -1;
    }
    
    // Publish the message to the drivers
    turn_in_place_publisher_.publish(command);
  }
    
  // Stop the robot from moving farther
  command.angular.z = 0.0;
  turn_in_place_publisher_.publish(command);
  
}

/*-----------------------------------------------------------------------------------*/

int main(int argc, char** argv) {

  // TODO: get rid of argc and argv???
  ros::init(argc, argv, "remote_teleop");
  
  ros::NodeHandle nh;
  
  ROS_INFO("main: instantiating an object of type RemoteTeleopClass");
  
  RemoteTeleopClass remote_teleop_class(&nh);
  
  ROS_INFO("main: going into spin; let the callbacks do all the work");
  
  ros::spin();
  
  return 0;
}
