#ifndef RT_BACKEND_H
#define RT_BACKEND_H

#include <stdlib.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

class RemoteTeleopClass {

public:

  RemoteTeleopClass();

private:

  // The ROS node handle.
  ros::NodeHandle nh_;
  
  // The tun in place ROS action server
  actionlib::SimpleActionServer<remote_teleop_robot_backend::TurnInPlaceAction> turn_in_place_server_;

  // The ROS publisher for the sending cmd_vel messages to the drivers
  ros::Publisher turn_in_place_publisher_;
  
  // The message used to publish the turn in place result
  remote_teleop_robot_backend::TurnInPlaceResult turn_in_place_result_;
  
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
  
  // Member methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeActions();

  void turn_in_place_callback(const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr& goal);
  void odom_callback(const nav_msgs::Odometry& msg);
  void turn_in_place();
};


#endif // RT_BACKEND_H
