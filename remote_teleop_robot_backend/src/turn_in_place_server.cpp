/* File: turn_in_place_server.cpp
 * Author: Anna Wong
 * Purpose: 
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <ros/rate.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include "turn_in_place_server.h"

/*-----------------------------------------------------------------------------------*/

// Define variables here
#define THRESHOLD 0.08
#define MIN_VEL 0.08

/*-----------------------------------------------------------------------------------*/

// CONSTRUCTOR: this will get called whenever an instance of this class is created
TurnInPlace::TurnInPlace(): turn_in_place_server_(nh_, "/turn_in_place_as", boost::bind(&TurnInPlace::turn_in_place_callback, this, _1), false) {

  ROS_INFO("In class constructor of TurnInPlace");
  
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

void TurnInPlace::initializeSubscribers() {
  
  ROS_INFO("Initializing Subscribers");
  
  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("/odom", 1, &TurnInPlace::odom_callback, this);

}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializePublishers() {

  ROS_INFO("Initializing Publishers");
  
  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializeActions() {

  ROS_INFO("Starting action servers");
  
  // Start the turn in place action server
  turn_in_place_server_.start();
  
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr& goal) {
  
  // TODO: gray out rviz plugin buttons when turn is being executed
  
  // Get inputs from Rviz and store them in variables
  angle_ = goal->degrees;
  turn_left_ = goal->turn_left;
    
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

void TurnInPlace::odom_callback(const nav_msgs::Odometry& msg) {
  
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

void TurnInPlace::turn_in_place() {

  ROS_INFO("Turn in Place Function");
  
  // Create message to be sent
  geometry_msgs::Twist command;
  
  // Set the unchanging fields
  // TODO: might not need this --> didn't need it in the python version
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  
  float goal_yaw = 0.0;
  

  if(turn_left_ == false) {
    // TURNING RIGHT
    goal_yaw = yaw_ - angle_;
    // Make sure the goal angle is within a valid range
    while(goal_yaw < -M_PI) {
      goal_yaw += 2*M_PI;
    }
  } else {
    // TURNING LEFT
    goal_yaw = yaw_ + angle_;
    // Make sure the goal angle is within a valid range
    while(goal_yaw > M_PI) {
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
    
    // Ensure the robot will never be turning at a speed greater than the desired angular velocity
    if (turn_left_ == true && command.angular.z > ang_vel_) {
      command.angular.z = ang_vel_;
    } else if (turn_left_ == false && abs(command.angular.z) > ang_vel_) {
      command.angular.z = -ang_vel_;
    } else if (turn_left_ == true && command.angular.z < MIN_VEL) {
      command.angular.z = MIN_VEL;
    } else if (turn_left_ == false && abs(command.angular.z) < MIN_VEL) {
      command.angular.z = -MIN_VEL;
    }
    
    // Publish the message to the drivers
    turn_in_place_publisher_.publish(command);
    
    ROS_INFO("%f, %f, %f", goal_yaw, yaw_ ,command.angular.z);
  }
  
  // Stop the robot once it has reached its goal
  command.angular.z = 0.0;
  turn_in_place_publisher_.publish(command);  
}

/*-----------------------------------------------------------------------------------*/

int main(int argc, char** argv) {

  ros::init(argc, argv, "remote_teleop");
   
  ROS_INFO("Main: instantiating an object of type TurnInPlace");
  
  TurnInPlace remote_teleop_class;
  
  ROS_INFO("Main: going into spin; let the callbacks do all the work");
  
  
  ros::spin();
  
  return 0;
}
