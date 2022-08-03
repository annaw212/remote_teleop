/* File: turn_in_place_server.cpp
 * Author: Anna Wong
 * Purpose: 
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include <remote_teleop_robot_backend/PointClickNavAction.h>
#include <remote_teleop_robot_backend/PointClickNavGoal.h>
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
  
  turn_in_place_running_ = false;
  point_and_click_running_ = false;

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
  
  // Set a variable to "claim" the drivers
  point_and_click_running_ = true;
  
  // TODO: check if turn in place is running
  
  // TODO: do something
  
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

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
}

/*-----------------------------------------------------------------------------------*/

int main(int argc, char** argv) {

  ros::init(argc, argv, "remote_teleop");
   
  ROS_INFO("Main: instantiating an object of type TurnInPlace");
  
  TurnInPlace remote_teleop_class;
  
  ROS_INFO("Main: creating interactive marker then going into spin");
  
  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();
  
  
  ros::spin();
  
  return 0;
}
