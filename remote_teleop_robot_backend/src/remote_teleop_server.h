#ifndef REMOTETELEOP_H
#define REMOTETELEOP_H

#include <ros/ros.h>

#include <cmath>
#include <mutex>
#include <stdlib.h>

#include <tf/tf.h>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include <remote_teleop_robot_backend/PointClickNavAction.h>
#include <remote_teleop_robot_backend/PointClickNavGoal.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>

#include <remote_teleop_robot_backend/SpeedToggleAction.h>
#include <remote_teleop_robot_backend/SpeedToggleGoal.h>
#include <remote_teleop_robot_backend/SpeedToggleResult.h>

#include <remote_teleop_robot_backend/StopNavAction.h>
#include <remote_teleop_robot_backend/StopNavGoal.h>
#include <remote_teleop_robot_backend/StopNavResult.h>

#include <remote_teleop_robot_backend/NudgeAction.h>
#include <remote_teleop_robot_backend/NudgeGoal.h>
#include <remote_teleop_robot_backend/NudgeResult.h>

#include <remote_teleop_robot_backend/ResetMarkerAction.h>
#include <remote_teleop_robot_backend/ResetMarkerGoal.h>
#include <remote_teleop_robot_backend/ResetMarkerResult.h>

class RemoteTeleop {

public:
  RemoteTeleop();

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
  actionlib::SimpleActionServer<remote_teleop_robot_backend::StopNavAction>
      stop_action_server_;
  actionlib::SimpleActionServer<remote_teleop_robot_backend::NudgeAction>
      nudge_server_;
  actionlib::SimpleActionServer<remote_teleop_robot_backend::SpeedToggleAction>
      velocity_server_;
  actionlib::SimpleActionServer<remote_teleop_robot_backend::ResetMarkerAction>
      reset_marker_server_;

  // ROS publishers
  ros::Publisher turn_in_place_publisher_;
  ros::Publisher point_click_nav_publisher_;
  ros::Publisher stop_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher occupancy_grid_debug_publisher_; // TODO: delete this
  ros::Publisher nudge_publisher_;
  ros::Publisher velocity_publisher_;

  // Result messages
  remote_teleop_robot_backend::TurnInPlaceResult turn_in_place_result_;
  remote_teleop_robot_backend::PointClickNavResult point_click_result_;
  remote_teleop_robot_backend::StopNavResult stop_nav_result_;
  remote_teleop_robot_backend::NudgeResult nudge_result_;
  remote_teleop_robot_backend::SpeedToggleResult velocity_result_;
  remote_teleop_robot_backend::ResetMarkerResult reset_marker_result_;

  // The ROS subscriber for receiving odometry value updates
  ros::Subscriber odom_sub_;
  ros::Subscriber costmap_sub_;

  // Internal variables
  float angle_;                                       // Turn in place angle
  bool turn_left_;                                    // Turn in place direction

  float lin_vel_;                                     // Max linear velocity
  float ang_vel_;                                     // Max angular velocity

  geometry_msgs::Pose current_odom_pose_;             // Odometry pose

  tfScalar roll_;                                     // Euler angles from odom
  tfScalar pitch_;
  tfScalar yaw_;

  geometry_msgs::Pose nav_goal_pose_;                 // Navigation goal pose

  std::string nav_goal_frame_;                        // Frame of navigation goal

  bool turn_in_place_running_;                        // State variables
  bool point_click_running_;
  bool obstacle_detected_;
  bool stop_;

  visualization_msgs::InteractiveMarker int_marker_;  // Interactive marker
  visualization_msgs::Marker marker_;                 // Regular marker TODO necessary?
  
  nav_msgs::OccupancyGrid occupancy_grid_;            // Occupancy grid
  
  tf2_ros::Buffer tf_buffer_;                          // Objects for frame transforms
  

  // Initialization member methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeActions();
  void initializeIntMarkers();
  void initializeFrontendVelocities(float lin_vel, float ang_vel);

  // Marker member methods
  visualization_msgs::Marker makeIntMarker();
  visualization_msgs::InteractiveMarkerControl &
  makeIntMarkerControl(visualization_msgs::InteractiveMarker &msg);

  // Callback member methods
  void turnInPlaceCallback(
      const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr &goal);
  void pointClickCallback(
      const remote_teleop_robot_backend::PointClickNavGoalConstPtr &msg);
  void
  stopNavCallback(const remote_teleop_robot_backend::StopNavGoalConstPtr &goal);
  void
  nudgeCallback(const remote_teleop_robot_backend::NudgeGoalConstPtr &goal);
  void speedToggleCallback(
      const remote_teleop_robot_backend::SpeedToggleGoalConstPtr &goal);
  void resetMarkerCallback(const remote_teleop_robot_backend::ResetMarkerGoalConstPtr &msg);

  // Subscriber callback methods
  void odomCallback(const nav_msgs::Odometry &msg);
  void costmapCallback(const nav_msgs::OccupancyGrid &grid);
  void processIntMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // Turn in place member methods
  void turnInPlace();

  // Point click navigate member methods
  void navigate(float angle, bool turn_left, float x_dist, float y_dist,
                float dist, float x2, float y2, float dx, float dy);
  void obstacleCheck(float x1, float y1, float x2, float y2, float dx, float dy,
                     bool smallSlope);

  // Stop nav member methods
  void stopMovement();

  // Nudge member methods
  void nudge(float x_dist, float y_dist, float dist);

  // Miscellaneous member methods
  geometry_msgs::PoseStamped transformGoalToOdom(geometry_msgs::Point &point,
                                                 const std::string init_frame,
                                                 const std::string goal_frame);
  geometry_msgs::Point
  translateCoordinateToCostmap(geometry_msgs::Point &point);
};

#endif // RemoteTeleop_H
