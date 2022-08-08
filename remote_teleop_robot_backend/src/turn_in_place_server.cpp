/* File: turn_in_place_server.cpp
 * Author: Anna Wong
 * Purpose: 
 */

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include <remote_teleop_robot_backend/PointClickNavAction.h>
#include <remote_teleop_robot_backend/PointClickNavGoal.h>
#include <remote_teleop_robot_backend/PointClickNavActionGoal.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>

#include "turn_in_place_server.h"

/*-----------------------------------------------------------------------------------*/

// Define variables here
#define THRESHOLD 0.03
#define MIN_VEL 0.08

/*-----------------------------------------------------------------------------------*/

// CONSTRUCTOR: this will get called whenever an instance of this class is created
TurnInPlace::TurnInPlace()
  : turn_in_place_server_(nh_, "/turn_in_place_as", boost::bind(&TurnInPlace::turn_in_place_callback, this, _1), false)
  , point_click_server_(nh_, "/point_click_as", boost::bind(&TurnInPlace::point_click_callback, this, _1), false)
{

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
  pos_x_ = 0.0;
  pos_y_ = 0.0;
  pos_z_ = 0.0;
  or_x_ = 0.0;
  or_y_ = 0.0;
  or_z_ = 0.0;
  or_w_ = 0.0;
  
  turn_in_place_running_ = false;
  point_and_click_running_ = false;

}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializeSubscribers() {
  
  ROS_INFO("Initializing Subscribers");
  
  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("/odom", 1, &TurnInPlace::odom_callback, this);
  
  nav_sub_ = nh_.subscribe("/geometry_msgs/Pose", 1, &TurnInPlace::test_callback, this);

}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializePublishers() {

  ROS_INFO("Initializing Publishers");
  
  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  
  // Initialize the point and click publisher
  point_click_nav_publisher_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 5);
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializeActions() {

  ROS_INFO("Starting action servers");
  
  // Start the turn in place action server
  turn_in_place_server_.start();
  
  // Start the point click action server
  point_click_server_.start();
  
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr& goal) {
  
  // TODO: gray out rviz plugin buttons when turn is being executed
  
  // Set a variable to "claim" the drivers
  turn_in_place_running_ = true;
  
  // TODO: check if point_and_click is running
  
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
  
  turn_in_place_running_ = false;
  
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::point_click_callback(const remote_teleop_robot_backend::PointClickNavGoalConstPtr& goal) {

  ROS_INFO("Point click callback function reached.");
  
  // Set a variable to "claim" the drivers
  point_and_click_running_ = true;
  
  // TODO: check if turn in place is running
  
  // Get the values from the goal
  pos_x_ = goal->goal_pose.position.x;
  pos_y_ = goal->goal_pose.position.y;
  pos_z_ = goal->goal_pose.position.z;
  or_x_ = goal->goal_pose.orientation.x;
  or_y_ = goal->goal_pose.orientation.y;
  or_z_ = goal->goal_pose.orientation.z;
  or_w_ = goal->goal_pose.orientation.w;
  
  ROS_INFO_STREAM("CB: " << pos_x_ << ", " << pos_y_ << ", " << pos_z_ << "\t" << or_x_ << ", " <<  or_y_ << ", " <<  or_z_ << ", " << or_w_);
  
  // Robot's self is always at 
  
  // Update the turn in place result and success fields
  point_click_result_.success = true;
  point_click_server_.setSucceeded(point_click_result_);
  
  point_and_click_running_ = false;
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

void TurnInPlace::test_callback(const visualization_msgs::InteractiveMarkerUpdate& msg) {
  ROS_INFO("Got here");
//  pos_x_ = msg.markers[0].pose.position.x;
//  pos_y_ = msg.markers[0].pose.position.y;
//  pos_z_ = msg.markers[0].pose.position.z;
//  or_x_ = msg.markers[0].pose.orientation.x;
//  or_y_ = msg.markers[0].pose.orientation.y;
//  or_z_ = msg.markers[0].pose.orientation.z;
//  or_w_ = msg.markers[0].pose.orientation.w;
 
//  ROS_INFO_STREAM(msg);
//  ROS_INFO_STREAM(pos_x_ << ", " << pos_y_ << ", " << pos_z_ << "\t" << or_x_ << ", " <<  or_y_ << ", " <<  or_z_ << ", " << or_w_);
  
  return;
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::turn_in_place() {
  
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
  }
  
  // Stop the robot once it has reached its goal
  command.angular.z = 0.0;
  turn_in_place_publisher_.publish(command);  
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::navigate() {

  ROS_INFO("NAVIGATE");
  // Calculate the a, b, c distance values between robot and goal
  float travel_dist;
  travel_dist = sqrt(pow(pos_x_, 2) + pow(pos_y_, 2));
  // Calculate angle to turn by to be facing goal location head on
  // if pos_x_ is negative, turn left, otherwise turn right
  
  // We first calculate the angle in the inner corner of the unit circle of the
  // triangle created by pos_x_, pos_y_, and travel_dist (hypoteneuse).
  // However, depending on whether we are on the top half or bottom half of the 
  // unit circle, this will mean we are either calculating the angle by which to turn,
  // OR the angle out of 90 degrees that we are not turning.
  // If we are on the bottom half, we are calculating the angle to turn, but because
  // it is on the bottom half, we also have to add 90 degrees because the robot
  // always considers itself to be pointed "forward". I hope this makes sense tmrw
  float theta1, theta2;
  theta1 = acos(pos_x_ / travel_dist);
  if(pos_y_ < 0.0) {
    theta1 += M_PI/2; // 90 degrees
  } else {
    theta1 = M_PI/2 - theta1;
  }
  // Calculate angle to turn by from goal to goal orientation
  // Grab the odometry quaternion values out of the message
  tf::Quaternion nav_q(
    or_x_,
    or_y_,
    or_z_,
    or_w_);
  
  tf::Matrix3x3 nav_mat(nav_q);
  
  tfScalar r, p, y;
    
  nav_mat.getRPY(r, p, y);
  
  if( y < 0.0) {
    // TURN RIGHT
    
    // TODO: angle_ = yaw_ + y OR yaw_ - y or y - yaw_?
  } else if( y > 0.0) {
    // TURN LEFT
    
  }
  
  // Navigate the robot to the desired location
  
  // Turn the robot the correct direction
  angle_ = theta1;
  
  
  
  
}

/*-----------------------------------------------------------------------------------*/

int main(int argc, char** argv) {

  ros::init(argc, argv, "remote_teleop");

  ROS_INFO("Main: instantiating an object of type TurnInPlace");
  
  TurnInPlace remote_teleop_class;
  
  ROS_INFO("Main: going into spin");
  
  
  ros::spin();
  
  return 0;
}
