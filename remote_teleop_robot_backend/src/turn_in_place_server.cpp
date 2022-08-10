/* File: turn_in_place_server.cpp
 * Author: Anna Wong
 * Purpose: 
 */

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>

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
  , point_click_server_(nh_, "/point_click_as", boost::bind(&TurnInPlace::nav_planning, this, _1), false)
  ,marker_server_("interactive_marker_server") {

  ROS_INFO("In class constructor of TurnInPlace");
  
  // Initialize the messy stuff
  initializeMarkers();
  initializeSubscribers();
  initializePublishers();
  initializeActions();  
  
  // Initialize the internal variables
  angle_ = 0.0;
  turn_left_ = true;
  lin_vel_ = 0.1;
  ang_vel_ = 0.5;
  x_ = 0.0;
  y_ = 0.0;
  z_ = 0.0;
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
  
//  nav_sub_ = nh_.subscribe("/geometry_msgs/Pose", 1, &TurnInPlace::test_callback, this);

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

visualization_msgs::Marker TurnInPlace::makeMarker() {
  
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 1.0;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  
  return marker;
}

/*-----------------------------------------------------------------------------------*/

visualization_msgs::InteractiveMarkerControl& TurnInPlace::makeMarkerControl(visualization_msgs::InteractiveMarker& msg) {

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeMarker() );
  msg.controls.push_back( control );
  
  return msg.controls.back();
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::initializeMarkers() {

  // Create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // Create the box marker and the non-interactive control which contains the box
  makeMarkerControl( int_marker );
  
  visualization_msgs::InteractiveMarkerControl control;
  
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  marker_server_.insert(int_marker, boost::bind(&TurnInPlace::processFeedback, this, _1));
//  marker_server_.setCallback(int_marker.name, boost::bind(&TurnInPlace::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  marker_server_.applyChanges();
  
  return;
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
//  lin_vel_ = 0.5;
//  ang_vel_ = 0.5;
//  
  // Tell robot to turn the desired angle
  turn_in_place();
  
  // Update the turn in place result and success fields
  turn_in_place_result_.success = true;
  turn_in_place_server_.setSucceeded(turn_in_place_result_);
  
  turn_in_place_running_ = false;
  
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback ) {
  
  pos_x_ = feedback->pose.position.x;
  pos_y_ = feedback->pose.position.y;
  pos_z_ = feedback->pose.position.z;
  
  or_x_ = feedback->pose.orientation.x;
  or_y_ = feedback->pose.orientation.y;
  or_z_ = feedback->pose.orientation.z;
  or_w_ = feedback->pose.orientation.w;
  
//  ROS_INFO_STREAM("CB: " << pos_x_ << ", " << pos_y_ << ", " << pos_z_ << "\t" << or_x_ << ", " <<  or_y_ << ", " <<  or_z_ << ", " << or_w_);
  // TODO: trigger navigation from these values
}
/*-----------------------------------------------------------------------------------*/

void TurnInPlace::point_click_callback() {

  ROS_INFO("Point click callback function reached.");
//  
////  // Set a variable to "claim" the drivers
////  point_and_click_running_ = true;
////  
////  // TODO: check if turn in place is running
////  
////  // Get the values from the goal
////  pos_x_ = goal->goal_pose.position.x;
////  pos_y_ = goal->goal_pose.position.y;
////  pos_z_ = goal->goal_pose.position.z;
////  or_x_ = goal->goal_pose.orientation.x;
////  or_y_ = goal->goal_pose.orientation.y;
////  or_z_ = goal->goal_pose.orientation.z;
////  or_w_ = goal->goal_pose.orientation.w;
////  
////  ROS_INFO_STREAM("CB: " << pos_x_ << ", " << pos_y_ << ", " << pos_z_ << "\t" << or_x_ << ", " <<  or_y_ << ", " <<  or_z_ << ", " << or_w_);
//  
//  // Robot's self is always at 
//  
//  // Update the turn in place result and success fields
//  point_click_result_.success = true;
//  point_click_server_.setSucceeded(point_click_result_);
//  
//  point_and_click_running_ = false;
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::odom_callback(const nav_msgs::Odometry& msg) {

  // Grab the odometry position values out of the message
  x_ = msg.pose.pose.position.x;
  y_ = msg.pose.pose.position.y;
  z_ = msg.pose.pose.position.z;
  
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

void TurnInPlace::nav_planning(const remote_teleop_robot_backend::PointClickNavGoalConstPtr& msg) {

  
  
  // Store values of position and orientation in local variables so they don't change during calculations
  float x = pos_x_;
  float y = pos_y_;
  float z = pos_z_;
  float a = or_x_;
  float b = or_y_;
  float c = or_z_;
  float d = or_w_;
  
  
  
  // TODO: might want to store the values of x, y, z, and orientation in local variables so they aren't being changed
  
  ROS_INFO("NAVIGATE");
  // Calculate the a, b, c distance values between robot and goal
  float travel_dist = 0.0;
  bool turn_left1 = true, turn_left2 = true;
  double theta1 = 0.0;
  tfScalar r, t, theta2 = 0.0;
  
  // Calculate the distance needed to travel
  // TODO: depending on how rviz considers the robot to be located, might need to use robot's x_, y_, z_ coords to calculate this hypoteneuse
  travel_dist = sqrt(pow(x, 2) + pow(y, 2));
  
  // Calculate the angle to turn in order to face the goal destination
  if (y < 0.0) {
    // Turning right
    turn_left1 = false;
    
    // Set the angle
    if (x > 0.0) {
      theta1 = acos(x / travel_dist);
    } else if (x < 0.0) {
      theta1 = acos(y / travel_dist);
//      ROS_INFO_STREAM(theta1 << "\t" << M_PI - theta1 << "\t" << acos(y/travel_dist));
    } else if (x == 0.0) {
      theta1 = M_PI/2;
    }
    
  } else if (y > 0.0) {
    // Turning left
    turn_left1 = true;
    
    // Set the angle
    if (x > 0.0) {
      theta1 = acos(x / travel_dist);
    } else if (x < 0.0) {
      theta1 = M_PI/2 + acos(y / travel_dist);
    } else if (x == 0.0) {
      theta1 = M_PI/2;
    }
    
  } else if (y == 0.0) {
    // Turning around, this variable can be set to anything
    turn_left1 = true;
    
    // Set the angle
    if (x < 0.0) {
      theta1 = M_PI;
    } else {
      theta1 = 0.0;
    }
  }
  
  theta1 = abs(theta1);
  ROS_INFO_STREAM(x << ", " << y << "\t" << x_ << ", " << y_ << "\t" << x_ + x << ", " << y_ + y);
  theta1 = atan2(y_, x_);
  ROS_INFO_STREAM(theta1);
  theta1 = 0;
  travel_dist = 0;
  
  // NAVIGATE

  // 1) Turn to face goal location
  navigate(theta1, turn_left1, 0.0, 0.0, 0.0);
//  // 2) Drive to goal location
  navigate(0.0, true, x, y, travel_dist);

  // Calculate angle to turn by from goal to goal orientation
  tf::Quaternion q(
    a,
    b,
    c,
    d);
  
  tf::Matrix3x3 m(q);
    
  m.getRPY(r, t, theta2);
  
  if( theta2 < 0.0) {
    // Turning right
    turn_left2 = false;
    
    // Set the angle
    theta2 = theta2 + yaw_;

  } else if( theta2 > 0.0) {
    // Turning left    
    turn_left2 = true;
    
    // Set the angle
    theta2 = theta2 - yaw_;
  }
//  // 3) Turn robot to goal orientation
  navigate(theta2, turn_left2, 0.0, 0.0, 0.0);
  
  
  ROS_INFO_STREAM("(" << x << ", " << y << ", " << z << ")" << "\t(" << a << ", " << b << ", " << c << ", " << d << ")" << "\t(" << r << ", " << t << ", " << theta2 << ")");
  ROS_INFO_STREAM("t1 = " << theta1 << "\tl1 = " << turn_left1 << "\tt2 = " << theta2 << "\tl2 = " << turn_left2 << "\tDist = " << travel_dist);
  
  // Update the turn in place result and success fields
  point_click_result_.success = true;
  point_click_server_.setSucceeded(point_click_result_);
}

/*-----------------------------------------------------------------------------------*/

void TurnInPlace::navigate(float angle, bool turn_left, float x_dist, float y_dist, float dist) {

  float goal_x, goal_y, start_x, start_y;
  
  // Create message to be sent
  geometry_msgs::Twist command;
  
  // Set the fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;
  
  if (angle == 0.0 && dist == 0.0) {
    ROS_INFO("DO NOTHING");
    // Do nothing
    return;
  }
  
  if (angle == 0.0) {
    ROS_INFO("DRIVE STRAIGHT");
    goal_x = x_ + x_dist;
    goal_y = y_ + y_dist;
    start_x = x_;
    start_y = y_;
    
    // Drive straight
    while (dist - (sqrt(pow(x_ - start_x, 2) + pow(y_ - start_y, 2))) > THRESHOLD) {
      // Set the linear velocity
      command.linear.x = std::min(lin_vel_ * abs((goal_x - x_)), lin_vel_ * abs((goal_y - y_)));
      
//      ROS_INFO_STREAM(dist - (sqrt(pow(x_ - start_x, 2) + pow(y_ - start_y, 2))));
      
      if (command.linear.x > lin_vel_) {
        command.linear.x = lin_vel_;
      } else if (command.linear.x < MIN_VEL) {
        command.linear.x = MIN_VEL;
      }
      // Publish the command
      point_click_nav_publisher_.publish(command);
    }
    // Stop the robot from moving
    command.linear.x = 0.0;
    point_click_nav_publisher_.publish(command);
  }
  
  if (dist == 0.0) {
    ROS_INFO("TURN IN PLACE");
    // Turn in place
    angle_ = angle;
    turn_left_ = turn_left;
    turn_in_place();
  }
  
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
