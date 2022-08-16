#ifndef TURNINPLACE_H
#define TURNINPLACE_H

#include <stdlib.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>

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
  actionlib::SimpleActionServer<remote_teleop_robot_backend::TurnInPlaceAction>
      turn_in_place_server_;
  actionlib::SimpleActionServer<
      remote_teleop_robot_backend::PointClickNavAction>
      point_click_server_;
  interactive_markers::InteractiveMarkerServer int_marker_server_;

  // ROS publishers
  ros::Publisher turn_in_place_publisher_;
  ros::Publisher point_click_nav_publisher_;

  // Result messages
  remote_teleop_robot_backend::TurnInPlaceResult turn_in_place_result_;
  remote_teleop_robot_backend::PointClickNavResult point_click_result_;

  // The ROS subscriber for receiving odometry value updates
  ros::Subscriber odom_sub_;

  // Internal variables
  float angle_;    // Turn in place angle
  bool turn_left_; // Turn in place direction

  float lin_vel_; // Max linear velocity
  float ang_vel_; // Max angular velocity

  tfScalar x_; // Odom position x
  tfScalar y_; // Odom position y
  tfScalar z_; // Odom position z
  tfScalar a_; // Odom orientation x
  tfScalar b_; // Odom orientation y
  tfScalar c_; // Odom orientation z
  tfScalar d_; // Odom orientation w

  tfScalar roll_; // Euler angles from odom
  tfScalar pitch_;
  tfScalar yaw_;

  float pos_x_; // Navigation goal position x
  float pos_y_; // Navigation goal position y
  float pos_z_; // Navigation goal position z
  float or_x_;  // Navigation goal orientation x
  float or_y_;  // Navigation goal orientation y
  float or_z_;  // Navigation goal orientation z
  float or_w_;  // Navigation goal orientation w

  bool turn_in_place_running_;
  bool point_and_click_running_;
  bool obstacle_detected_;

  // Initialization member methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeActions();
  void initializeIntMarkers();
  void initializeMarkers();

  // Marker member methods
  visualization_msgs::Marker makeMarker();
  visualization_msgs::Marker makeIntMarker();
  visualization_msgs::InteractiveMarkerControl &
  makeIntMarkerControl(visualization_msgs::InteractiveMarker &msg);

  // Callback member methods
  void turn_in_place_callback(
      const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr &goal);
  void point_click_callback(
      const remote_teleop_robot_backend::PointClickNavGoalConstPtr &msg);
  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void odom_callback(const nav_msgs::Odometry &msg);

  // Turn in place member methods
  void turn_in_place();

  // Point click navigate member methods
  void navigate(float angle, bool turn_left, float x_dist, float y_dist,
                float dist);
  void obstacle_check(int x1, int y1, int x2, int y2, int dx, int dy,
                      bool smallSlope);
};

#endif // TURNINPLACE_H
