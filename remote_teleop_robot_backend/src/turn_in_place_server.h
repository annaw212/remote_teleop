#ifndef TURNINPLACE_H
#define TURNINPLACE_H

#include <stdlib.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include <remote_teleop_robot_backend/PointClickNavAction.h>
#include <remote_teleop_robot_backend/PointClickNavGoal.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>

class TurnInPlace {

public:

  TurnInPlace();

private:

  // The ROS node handle.
  ros::NodeHandle nh_;
  
  // ROS action servers
  actionlib::SimpleActionServer<remote_teleop_robot_backend::TurnInPlaceAction> turn_in_place_server_;
  actionlib::SimpleActionServer<remote_teleop_robot_backend::PointClickNavAction> point_click_server_;

  // ROS publishers
  ros::Publisher turn_in_place_publisher_;
  ros::Publisher point_click_nav_publisher_;
  
  // Result messages
  remote_teleop_robot_backend::TurnInPlaceResult turn_in_place_result_;
  remote_teleop_robot_backend::PointClickNavResult point_click_result_;
  
  // The ROS subscriber for receiving odometry value updates
  ros::Subscriber odom_sub_;

  // Internal variables
  float angle_;
  bool turn_left_;
  float lin_vel_;
  float ang_vel_;
  tfScalar roll_;
  tfScalar pitch_;
  tfScalar yaw_;
  float pos_x_;
  float pos_y_;
  float pos_z_;
  float or_x_;
  float or_y_;
  float or_z_;
  float or_w_;
  
  bool turn_in_place_running_;
  bool point_and_click_running_;
  
  // Member methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeActions();

  void turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr& goal);
  void point_click_callback(const remote_teleop_robot_backend::PointClickNavGoalConstPtr& goal);
  void odom_callback(const nav_msgs::Odometry& msg);
  void turn_in_place();
  void navigate();
};


#endif // TURNINPLACE_H
