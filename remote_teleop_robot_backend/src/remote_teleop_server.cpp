/* File: remote_teleop_server.cpp
 * Author: Anna Wong
 * Purpose:
 */
#include <ros/ros.h>

#include <array>
#include <cmath>

#include <tf/tf.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>

#include <remote_teleop_robot_backend/Velocity.h>

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

#include "remote_teleop_robot_backend/remote_teleop_server.h"

/*----------------------------------------------------------------------------------*/

// Define variables here
#define THRESHOLD 0.03
#define MIN_VEL 0.08
#define ANGLE_THRESHOLD 0.035
#define INIT_LIN_VEL 0.5
#define INIT_ANG_VEL 1.0

/*----------------------------------------------------------------------------------*/

// CONSTRUCTOR: this will get called whenever an instance of this class is
// created
RemoteTeleop::RemoteTeleop()
    : turn_in_place_server_(
          nh_, "/turn_in_place_as",
          boost::bind(&RemoteTeleop::turnInPlaceCallback, this, _1), false),
      point_click_server_(
          nh_, "/point_click_as",
          boost::bind(&RemoteTeleop::pointClickCallback, this, _1), false),
      int_marker_server_("interactive_marker_server"),
      stop_action_server_(nh_, "/stop_nav_as",
                          boost::bind(&RemoteTeleop::stopNavCallback, this, _1),
                          false),
      nudge_server_(nh_, "/nudge_as",
                    boost::bind(&RemoteTeleop::nudgeCallback, this, _1), false),
      velocity_server_(
          nh_, "/speed_toggle_as",
          boost::bind(&RemoteTeleop::speedToggleCallback, this, _1), false),
      reset_marker_server_(
          nh_, "/reset_marker_as",
          boost::bind(&RemoteTeleop::resetMarkerCallback, this, _1), false) {

  // Initialize the messy stuff
  initializeIntMarkers();
  initializeSubscribers();
  initializePublishers();
  initializeActions();

  // Initialize the internal variables

  lin_vel_ = INIT_LIN_VEL; // Velocity
  ang_vel_ = INIT_ANG_VEL;

  current_odom_pose_.pose.position.x = 0.0; // Current odometry pose
  current_odom_pose_.pose.position.y = 0.0;
  current_odom_pose_.pose.position.z = 0.0;
  current_odom_pose_.pose.orientation.w = 0.0;
  current_odom_pose_.pose.orientation.x = 0.0;
  current_odom_pose_.pose.orientation.y = 0.0;
  current_odom_pose_.pose.orientation.z = 0.0;

  nav_goal_pose_.pose.position.x = 0.0; // Navigation goal pose
  nav_goal_pose_.pose.position.y = 0.0;
  nav_goal_pose_.pose.position.z = 0.0;
  nav_goal_pose_.pose.orientation.w = 0.0;
  nav_goal_pose_.pose.orientation.x = 0.0;
  nav_goal_pose_.pose.orientation.y = 0.0;
  nav_goal_pose_.pose.orientation.z = 0.0;

  turn_in_place_running_ = false; // State booleans
  point_click_running_ = false;
  obstacle_detected_ = false;
  stop_ = false;

  // Send the initial velocities to rviz
  initializeFrontendVelocities(lin_vel_, ang_vel_);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeSubscribers() {

  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("/odom", 1, &RemoteTeleop::odomCallback, this);

  // Initialize the costmap subscriber
  costmap_sub_ = nh_.subscribe("/rt_costmap_node/costmap/costmap", 1,
                               &RemoteTeleop::costmapCallback, this);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::initializePublishers() {

  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the point and click publisher
  point_click_nav_publisher_ =
      nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the stop publisher
  stop_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the marker publisher
  marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Initialize the occupancy grid debug publisher
  occupancy_grid_debug_publisher_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_debug", 5);

  // Initialize the nudge publisher
  nudge_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the velocity publisher
  velocity_publisher_ = nh_.advertise<remote_teleop_robot_backend::Velocity>(
      "rt_initial_velocities", 1);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeActions() {

  // Start the turn in place action server
  turn_in_place_server_.start();

  // Start the point click action server
  point_click_server_.start();

  // Start stop nav action server
  stop_action_server_.start();

  // Start nudge action server
  nudge_server_.start();

  // Start speed toggle action server
  velocity_server_.start();

  // Start the reset marker action server
  reset_marker_server_.start();
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeIntMarkers() {

  // Create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // Create the box marker and the non-interactive control containing the box
  makeIntMarkerControl(int_marker);

  // Create the interactive marker control
  visualization_msgs::InteractiveMarkerControl control;

  // Set the control for movement along the x-axis
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Set the control for movement along the y-axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Set the control for movement about the z-axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  int_marker_ = int_marker;

  // Add the interactive marker to our collection & tell the server to call
  // processIntMarkerFeedback() when feedback arrives for it
  int_marker_server_.insert(
      int_marker_,
      boost::bind(&RemoteTeleop::processIntMarkerFeedback, this, _1));

  // 'commit' changes and send to all clients
  int_marker_server_.applyChanges();

  return;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeFrontendVelocities(float lin_vel, float ang_vel) {

  // Create message
  remote_teleop_robot_backend::Velocity init_vel_msg;

  // Assign values to message fields
  init_vel_msg.lin_vel = lin_vel;
  init_vel_msg.ang_vel = lin_vel;

  // Publish the message
  velocity_publisher_.publish(init_vel_msg);
}

/*----------------------------------------------------------------------------------*/

visualization_msgs::Marker RemoteTeleop::makeIntMarker() {
  // Create a marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  // Use mesh
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Scale the marker
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  // Assign colors to the marker
  marker.color.r = 0.1;
  marker.color.g = 0.56;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.mesh_use_embedded_materials = false;

  // Add the mesh
  marker.mesh_resource = "package://freight_100_description/meshes/freight.dae";
  marker.pose.orientation.w = 1;

  return marker;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::makeIntMarkerControl(visualization_msgs::InteractiveMarker &msg) {

  // Create an interactive marker control
  visualization_msgs::InteractiveMarkerControl control;
  // Set the control variables
  control.always_visible = true;
  // Assign a marker to the control
  control.markers.push_back(makeIntMarker());
  // Assign the control to an interactive marker
  msg.controls.push_back(control);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::placeGoalMarker() {

  // Set the goal frame
  std::string goal_frame = "odom";

  // Create non-interactive markers
  visualization_msgs::Marker sphere_marker;
  visualization_msgs::Marker cylinder_marker;

  // Give the markers header values and a namespace
  sphere_marker.header.frame_id = goal_frame;
  cylinder_marker.header.frame_id = goal_frame;
  sphere_marker.header.stamp = ros::Time::now();
  cylinder_marker.header.stamp = ros::Time::now();
  sphere_marker.ns = "remote_teleop_goal_marker";
  cylinder_marker.ns = "remote_teleop_goal_marker";
  sphere_marker.id = 0;
  cylinder_marker.id = 1;

  // Assign shapes to the markers and add the markers to rviz
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  cylinder_marker.type = visualization_msgs::Marker::CYLINDER;

  sphere_marker.action = visualization_msgs::Marker::ADD;
  cylinder_marker.action = visualization_msgs::Marker::ADD;

  // Create a PoseStamped message to store the pose goal of the marker in
  geometry_msgs::PoseStamped marker_pose = nav_goal_pose_;

  // Make sure the markers' goal poses are in odom
  marker_pose = transformPose(nav_goal_pose_, goal_frame);

  // Scale the markers
  sphere_marker.scale.x = 0.4;
  sphere_marker.scale.y = 0.4;
  sphere_marker.scale.z = 0.4;

  cylinder_marker.scale.x = 0.1;
  cylinder_marker.scale.y = 0.1;
  cylinder_marker.scale.z = 0.9;

  // Assign the pose values to the marker
  sphere_marker.pose.position.x = marker_pose.pose.position.x;
  sphere_marker.pose.position.y = marker_pose.pose.position.y;
  sphere_marker.pose.position.z = marker_pose.pose.position.z + cylinder_marker.scale.z;
  sphere_marker.pose.orientation.x = 0.0;
  sphere_marker.pose.orientation.y = 0.0;
  sphere_marker.pose.orientation.z = 0.0;
  sphere_marker.pose.orientation.w = 1.0;

  cylinder_marker.pose.position.x = marker_pose.pose.position.x;
  cylinder_marker.pose.position.y = marker_pose.pose.position.y;
  cylinder_marker.pose.position.z = marker_pose.pose.position.z + cylinder_marker.scale.z / 2.0;
  cylinder_marker.pose.orientation.x = 0.0;
  cylinder_marker.pose.orientation.y = 0.0;
  cylinder_marker.pose.orientation.z = 0.0;
  cylinder_marker.pose.orientation.w = 1.0;

  // Give the markers colors
  sphere_marker.color.r = 1.0;
  sphere_marker.color.g = 0.0;
  sphere_marker.color.b = 0.0;
  sphere_marker.color.a = 1.0;

  cylinder_marker.color.r = 0.9;
  cylinder_marker.color.g = 0.9;
  cylinder_marker.color.b = 0.9;
  cylinder_marker.color.a = 1.0;

  // Set the lifetime of the markers; ros::Duration() = forever
  sphere_marker.lifetime = ros::Duration();
  cylinder_marker.lifetime = ros::Duration();

  // Publish the markers
  marker_publisher_.publish(sphere_marker);
  marker_publisher_.publish(cylinder_marker);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::deleteGoalMarker() {

  // Create a non-interactive marker
  visualization_msgs::Marker marker;
  // Tell the markers to delete themselves
  marker.action = visualization_msgs::Marker::DELETEALL;
  // Set a duration
  marker.lifetime = ros::Duration();
  // Publish the marker
  marker_publisher_.publish(marker);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::turnInPlaceCallback(
    const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr &goal) {

  // TODO: gray out rviz plugin buttons when turn is being executed

  // Set a variable to signal that turning in place is currently happening
  turn_in_place_running_ = true;

  // Delete the marker
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  // Tell robot to turn the desired angle
  turnInPlace(goal->degrees * (M_PI / 180), goal->turn_left);

  // Create a new marker
  initializeIntMarkers();

  // Update the turn in place result and success fields
  turn_in_place_result_.success = true;
  turn_in_place_server_.setSucceeded(turn_in_place_result_);

  // Set a variable to signal that turning in place has ended
  turn_in_place_running_ = false;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::processIntMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  // Grab the incoming position info from the marker
  nav_goal_pose_.header = feedback->header;
  nav_goal_pose_.pose = feedback->pose;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::odomCallback(const nav_msgs::Odometry &msg) {

  // Grab the odometry position and orientation values out of the message
  current_odom_pose_.header = msg.header;
  current_odom_pose_.pose = msg.pose.pose;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::costmapCallback(const nav_msgs::OccupancyGrid &grid) {

  // Store the values of the occupancy grid in a variable for future reference
  occupancy_grid_ = grid;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::resetMarkerCallback(
    const remote_teleop_robot_backend::ResetMarkerGoalConstPtr &msg) {
  // Delete the marker and then create a new one
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  initializeIntMarkers();

  // Update the reset marker result and success fields
  reset_marker_result_.success = true;
  reset_marker_server_.setSucceeded(reset_marker_result_);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::nudgeCallback(
    const remote_teleop_robot_backend::NudgeGoalConstPtr &goal) {

  // Delete the marker
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  // Initiate the nudge command
  if (goal->fwd == true && !stop_) {
    // Nudge forward
    nudge(goal->dist, 0.0, goal->dist);
  } else if (goal->fwd == false && !stop_) {
    // Nudge backward
    nudge(-1 * goal->dist, 0.0, -1 * goal->dist);
  } else {
    // Stop the robot
    stopMovement();
    stop_ = false;
  }

  // Update the nudge result and success fields
  nudge_result_.success = true;
  nudge_server_.setSucceeded(nudge_result_);

  // Initialize the markers
  initializeIntMarkers();
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::speedToggleCallback(
    const remote_teleop_robot_backend::SpeedToggleGoalConstPtr &goal) {

  // Update the velocity variables
  lin_vel_ = goal->lin_vel;
  ang_vel_ = goal->ang_vel;

  // Update the velocity result and success fields
  velocity_result_.success = true;
  velocity_server_.setSucceeded(velocity_result_);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::turnInPlace(float angle, bool turn_left) {

  // Create message to be sent
  geometry_msgs::Twist command;

  // Set the fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  // Create the goal yaw variable
  float goal_yaw = 0.0;

  // Calculate current yaw angle
  std::array<tfScalar, 3> curr_angle;
  curr_angle = eulerFromQuaternion(current_odom_pose_.pose);

  if (turn_left == false) {
    // TURNING RIGHT
    goal_yaw = curr_angle[2] - angle;
    // Make sure the goal angle is within a valid range
    while (goal_yaw < -M_PI) {
      goal_yaw += 2 * M_PI;
    }
    // Since 180 is always a left turn in navigate, need to make sure it's not
    // out of bounds
    while (goal_yaw > M_PI) {
      goal_yaw -= 2 * M_PI;
    }

  } else {
    // TURNING LEFT
    goal_yaw = curr_angle[2] + angle;
    // Make sure the goal angle is within a valid range
    while (goal_yaw > M_PI) {
      goal_yaw -= 2 * M_PI;
    }
    // Since 180 is always a left turn in navigate, need to make sure it's not
    // out of bounds
    while (goal_yaw < -M_PI) {
      goal_yaw += 2 * M_PI;
    }
  }

  // Turn the robot until it reaches the desired angle
  while (abs(goal_yaw - curr_angle[2]) > THRESHOLD && !stop_) {

    // Calculate current yaw angle
    curr_angle = eulerFromQuaternion(current_odom_pose_.pose);

    // Set the turn rate
    command.angular.z = ang_vel_ * (goal_yaw - curr_angle[2]);

    // Ensure the robot will be turning in the correct direction
    if (turn_left == true && command.angular.z < 0.0) {
      command.angular.z *= -1;
    } else if (turn_left == false && command.angular.z > 0.0) {
      command.angular.z *= -1;
    }

    // Ensure the robot will never be turning at a speed greater than the
    // desired angular velocity
    if (turn_left == true && command.angular.z > ang_vel_) {
      command.angular.z = ang_vel_;
    } else if (turn_left == false && abs(command.angular.z) > ang_vel_) {
      command.angular.z = -ang_vel_;
    } else if (turn_left == true && command.angular.z < MIN_VEL) {
      command.angular.z = MIN_VEL;
    } else if (turn_left == false && abs(command.angular.z) < MIN_VEL) {
      command.angular.z = -MIN_VEL;
    }

    // Publish the message to the drivers
    turn_in_place_publisher_.publish(command);
  }

  // Stop the robot once it has reached its goal
  command.angular.z = 0.0;
  turn_in_place_publisher_.publish(command);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::pointClickCallback(
    const remote_teleop_robot_backend::PointClickNavGoalConstPtr &msg) {

  // Set a variable to signal that the robot is navigating
  point_click_running_ = true;

  // Store values of position and orientation in local variables so they don't
  // change during calculations
  geometry_msgs::PoseStamped goal_pose;
  goal_pose =
      nav_goal_pose_; // in relation to the robot, which thinks it's at (0,0,0)

  // Put that into base link
  // NOTE: The reason I am not using the transformPose() function is because
  // inside that function, the orientation variables are reset to (1,0,0,0)
  // before the transform which messes up the base link values. Therefore, due
  // to a lack of time, I am just leaving this transform here and doing the rest
  // of the transforms in my helper function.
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped nav_goal_frame_to_base_link;

  nav_goal_frame_to_base_link =
      tf_buffer.lookupTransform("base_link", nav_goal_pose_.header.frame_id,
                                ros::Time(0), ros::Duration(1.0));

  // Input the point you want to transform and indicate we want to just
  // overwrite that object with the transformed point values
  geometry_msgs::PoseStamped base_link_goal;
  std::string goal_frame = "base_link";
  base_link_goal = goal_pose;
  tf2::doTransform(base_link_goal, base_link_goal, nav_goal_frame_to_base_link);
  //  base_link_goal = transformPose(goal_pose, goal_frame);

  // Declare local variables
  float travel_dist = 0.0;
  bool turn_left1 = true, turn_left2 = true;
  double theta1 = 0.0;
  tfScalar r, t, theta2 = 0.0;

  // Calculate the distance needed to travel
  travel_dist = sqrt(
      pow(abs(goal_pose.pose.position.x - current_odom_pose_.pose.position.x),
          2) +
      pow(abs(goal_pose.pose.position.y - current_odom_pose_.pose.position.y),
          2));

  // Calculate the angle needed to turn to face goal point
  // TODO: figure out a better way to do this
  if (abs(current_odom_pose_.pose.position.x - goal_pose.pose.position.x) <=
          0.001 &&
      abs(current_odom_pose_.pose.position.y - goal_pose.pose.position.y) <=
          0.001) {
    theta1 = 0;
  } else {
    theta1 =
        atan2(base_link_goal.pose.position.y, base_link_goal.pose.position.x);
  }

  // Edge case checking
  if (abs(theta1) <= ANGLE_THRESHOLD) {
    // If we are only turning 1-3 degrees, just don't turn
    theta1 = 0;
  } else if (abs(M_PI - theta1) <= ANGLE_THRESHOLD) {
    // If we are turning to an angle within 1-3 degrees of a 180 turn, turn 180
    theta1 = M_PI;
  }

  /* DETERMINE VALIDITY OF THE PATH */

  // Set the goal coordinates on the costmap grid
  geometry_msgs::Point curr_coords, goal_coords;
  curr_coords.x = current_odom_pose_.pose.position.x;
  curr_coords.y = current_odom_pose_.pose.position.y;
  goal_coords.x = goal_pose.pose.position.x; // TODO used to be goal_pose
  goal_coords.y = goal_pose.pose.position.y;

  // Transform goal to odom
  geometry_msgs::PoseStamped pose;
  pose.header = goal_pose.header;
  pose.pose.position.x = goal_coords.x;
  pose.pose.position.y = goal_coords.y;
  goal_frame = "odom";
  pose = transformPose(pose, goal_frame);

  // Translate current and goal coordinates to costmap
  curr_coords = translateCoordinateToCostmap(curr_coords);
  goal_coords = translateCoordinateToCostmap(
      pose.pose.position); // used to be goal_coords

  // Extract the values of the coordinates into int variables
  int curr_x = curr_coords.x;
  int curr_y = curr_coords.y;
  int goal_x = goal_coords.x;
  int goal_y = goal_coords.y;

  // Find the slope between the points to determine which direction we need to
  // draw the Brensenham's line in
  float dx = abs(goal_x - curr_x);
  float dy = abs(goal_y - curr_y);

  if (abs(current_odom_pose_.pose.position.x - goal_pose.pose.position.x) <=
          0.001 &&
      abs(current_odom_pose_.pose.position.y - goal_pose.pose.position.y) <=
          0.001) {
    if (dx > dy) {
      // Slope is less than 1
      obstacleCheck(curr_x, curr_y, goal_x, goal_y, dx, dy, true);
    } else {
      // Slope is greater than 1
      obstacleCheck(curr_x, curr_y, goal_x, goal_y, dx, dy, false);
    }
  }

  if (obstacle_detected_ == true) {
    // Path was not clear -- reset variable and exit function
    obstacle_detected_ = false;
    // Update the turn in place result and success fields
    point_click_result_.success = true;
    point_click_server_.setSucceeded(point_click_result_);
    // Snap the interactive marker back to (0,0,0)
    initializeIntMarkers();
    return;
  }

  ROS_INFO("SAFE TO NAVIGATE"); // TODO: Delete this

  // TODO: Delete the interactive marker so it's not confusing during navigation
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  placeGoalMarker();

  // Determine direction to turn, and turn to face goal location
  // The reason for having the navigation command inside this function instead
  // of having it be like the others, is because we need to change the value of
  // theta1 to be positive if we are turning right, but theta2 is based on
  // theta1's original value, so this is _one_ way to make sure theta1 can keep
  // its original value...

  if (!stop_) {

    /* NAVIGATE */

    // Turn to face goal location
    if (theta1 < 0.0) {
      turn_left1 = false;
      navigate(theta1 * -1, turn_left1, 0.0, 0.0, 0.0, goal_x, goal_y, dx, dy);
    } else {
      turn_left1 = true;
      navigate(theta1, turn_left1, 0.0, 0.0, 0.0, goal_x, goal_y, dx, dy);
    }

    // Drive straight to goal location
    navigate(0.0, true, goal_pose.pose.position.x, goal_pose.pose.position.y,
             travel_dist, goal_x, goal_y, dx, dy);

    // Calculate angle to turn by from goal to goal orientation
    std::array<tfScalar, 3> angle;
    angle = eulerFromQuaternion(base_link_goal.pose);

    // Because theta2 is simply the angle to turn based on the original
    // orientation, we need to shift the degrees to turn appropriately
    theta2 = angle[2] - theta1;

    // Determine direction to turn
    if (theta2 < 0.0) {
      // Turn right
      turn_left2 = false;
      // Make sure any angle being sent to the robot is positive
      theta2 *= -1;
    } else {
      // Turn left
      turn_left2 = true;
    }

    // Turn robot to goal orientation
    navigate(theta2, turn_left2, 0.0, 0.0, 0.0, goal_x, goal_y, dx, dy);

  } else {
    stopMovement();
  }

  // Update the turn in place result and success fields
  point_click_result_.success = true;
  point_click_server_.setSucceeded(point_click_result_);

  // Snap the interactive marker back to (0,0,0)
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  // Delete the goal marker and reinsert an interactive marker
  deleteGoalMarker();
  initializeIntMarkers();

  // Set a variable to signal that navigation is complete
  point_click_running_ = false;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::navigate(float angle, bool turn_left, float x_dist,
                            float y_dist, float dist, float x2, float y2,
                            float dx, float dy) {

  if (stop_) {
    stopMovement();
    return;
  }

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
    // Do nothing
    return;
  }

  if (angle == 0.0) {
    // Determine goal coordinates
    geometry_msgs::Point start, goal;
    start.x = current_odom_pose_.pose.position.x;
    start.y = current_odom_pose_.pose.position.y;
    goal.x = start.x + x_dist;
    goal.y = start.y + y_dist;
    // Drive straight
    while (
        abs(dist) -
                (sqrt(pow(current_odom_pose_.pose.position.x - start.x, 2) +
                      pow(current_odom_pose_.pose.position.y - start.y, 2))) >
            THRESHOLD &&
        !stop_) {

      // Check for obstacles
      geometry_msgs::Point start_point;
      start_point.x = current_odom_pose_.pose.position.x;
      start_point.y = current_odom_pose_.pose.position.y;

      // Translate the current coordinates to the costmap
      start_point = translateCoordinateToCostmap(start_point);

      // Extract the values of the costmap coordinates into ints
      int x1 = start_point.x;
      int y1 = start_point.y;

      // Calculate the "slope" between the two points
      float dx = abs(x2 - x1);
      float dy = abs(y2 - y1);

      if (dx > dy) {
        // Slope is less than 1
        obstacleCheck(x1, y1, x2, y2, dx, dy, true);
      } else {
        // Slope is greater than 1
        obstacleCheck(x1, y1, x2, y2, dx, dy, false);
      }

      if (obstacle_detected_ == true) {
        // Path was not clear -- reset variable and exit function
        obstacle_detected_ = false;
        // Stop the robot from moving
        stopMovement();
        // Snap the interactive marker back to (0,0,0)
        initializeIntMarkers();
        return;
      }

      // Set the linear velocity
      command.linear.x = std::min(
          lin_vel_ * abs((goal.x - current_odom_pose_.pose.position.x)),
          lin_vel_ * abs((goal.y - current_odom_pose_.pose.position.y)));
      // Make sure the velocity is within the set bounds
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
    return;
  }

  if (dist == 0.0) {
    // Turn in place
    //    angle_ = angle;
    //    turn_left_ = turn_left;
    turnInPlace(angle, turn_left);

    return;
  }
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::stopNavCallback(
    const remote_teleop_robot_backend::StopNavGoalConstPtr &goal) {

  // Set the internal variable to signal that the stop button has been pressed
  stop_ = goal->stop;

  // Delete the markers
  int_marker_server_.clear();
  int_marker_server_.applyChanges();

  while (turn_in_place_running_ || point_click_running_) {
    // Keep the stop signal alive until point-click and turn-in-place are
    // finished
  }

  // Set the internal variable to false to allow for navigation commands to be
  // received again
  stop_ = false;

  // Initialize the markers
  initializeIntMarkers();

  // Set the result and success fields
  stop_nav_result_.success = true;
  stop_action_server_.setSucceeded(stop_nav_result_);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::stopMovement() {

  // Create message to be sent
  geometry_msgs::Twist command;

  // Set the fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  // Publish the message
  stop_publisher_.publish(command);
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::obstacleCheck(float x1, float y1, float x2, float y2,
                                 float dx, float dy, bool smallSlope) {
  // Using Brensenham's line algorithm to produce the straight-line coordinates
  // between two points. Taking those points and checking their locations on the
  // obstacle grid to make sure there are no obstacles in the way of navigation.

  // Create all the necessary variables
  int idx;

  // Brensenham's line algorithm -- geeks4geeks
  int pk = 2 * dy - dx;

  for (int h = 0; h <= dx; h++) {

    // Find the index of the point we are currently looking at
    idx = ceil(y1 * occupancy_grid_.info.width + x1);

    // Check if there is an obstacle at that point
    if (occupancy_grid_.data[idx] != 0) {
      ROS_INFO("OBSTACLE DETECTED");
      // Set the variable to indicate that an obstacle has been detected
      obstacle_detected_ = true;
      return;
    }

    // checking either to decrement or increment the value
    // if we have to plot from (0,100) to (100,0)
    x1 < x2 ? x1++ : x1--;

    if (pk < 0) {
      // decision value will decide to plot
      // either  x1 or y1 in x's position
      if (smallSlope == true) {
        pk = pk + 2 * dy;
      } else {
        //(y1,x1) is passed in xt
        pk = pk + 2 * dy;
      }
    } else {
      y1 < y2 ? y1++ : y1--;
      pk = pk + 2 * dy - 2 * dx;
    }
  }
}

/*----------------------------------------------------------------------------------*/

geometry_msgs::Point
RemoteTeleop::translateCoordinateToCostmap(geometry_msgs::Point &point) {

  // Shift the points by the offset of the costmap from odom
  point.x -= occupancy_grid_.info.origin.position.x;
  point.y -= occupancy_grid_.info.origin.position.y;

  // Scale the coordinates by the resolution
  point.x = ceil(point.x / occupancy_grid_.info.resolution);
  point.y = ceil(point.y / occupancy_grid_.info.resolution);

  return point;
}

/*----------------------------------------------------------------------------------*/

geometry_msgs::PoseStamped
RemoteTeleop::transformPose(geometry_msgs::PoseStamped pose,
                            const std::string goal_frame) {

  // Create all the necessary variables
  geometry_msgs::PoseStamped robot_pose;

  // Set the robot_pose values --> these are the values of our goal since that
  // is the point that needs to be converted from base link to odom frame
  robot_pose.pose.position.x = pose.pose.position.x;
  robot_pose.pose.position.y = pose.pose.position.y;
  robot_pose.pose.position.z = 0;
  robot_pose.pose.orientation.w = 1.0;

  // Create the objects needed for the transform
  // TODO: make the listener an internal variable
  tf2_ros::TransformListener tf2_listener(tf_buffer_);
  geometry_msgs::TransformStamped init_to_goal_frame;

  // Lookup the transform from the initial frame to odom and store in variable
  init_to_goal_frame = tf_buffer_.lookupTransform(
      goal_frame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0));

  // Input the point you want to transform and indicate we want to just
  // overwrite that object with the transformed point values
  tf2::doTransform(robot_pose, robot_pose,
                   init_to_goal_frame); // robot_pose is the PoseStamped I
                                        // want to transform

  return robot_pose;
}

/*----------------------------------------------------------------------------------*/

void RemoteTeleop::nudge(float x_dist, float y_dist, float dist) {

  if (stop_) {
    stopMovement();
    return;
  }

  // Create message to be sent
  geometry_msgs::Twist command;

  // Set the fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  // Determine goal coordinates
  geometry_msgs::Point start, goal;
  start.x = current_odom_pose_.pose.position.x;
  start.y = current_odom_pose_.pose.position.y;
  goal.x = start.x + x_dist;
  goal.y = start.y + y_dist;

  // Drive straight
  while (abs(dist) -
                 (sqrt(pow(current_odom_pose_.pose.position.x - start.x, 2) +
                       pow(current_odom_pose_.pose.position.y - start.y, 2))) >
             THRESHOLD &&
         !stop_) {

    // Set the linear velocity
    command.linear.x =
        std::min(lin_vel_ * abs((goal.x - current_odom_pose_.pose.position.x)),
                 lin_vel_ * abs((goal.y - current_odom_pose_.pose.position.y)));
    // Make sure the velocity is within the set bounds
    if (command.linear.x > lin_vel_) {
      command.linear.x = lin_vel_;
    } else if (command.linear.x < MIN_VEL) {
      command.linear.x = MIN_VEL;
    }
    // Make sure the velocity is negative if the direction is backwards
    if (dist < 0.0) {
      command.linear.x *= -1;
    }

    // Publish the command
    point_click_nav_publisher_.publish(command);
  }
  // Stop the robot from moving
  command.linear.x = 0.0;
  point_click_nav_publisher_.publish(command);
}

/*----------------------------------------------------------------------------------*/

std::array<tfScalar, 3>
RemoteTeleop::eulerFromQuaternion(geometry_msgs::Pose &angle) {

  // Create array to store values in
  std::array<tfScalar, 3> arr;

  // Grab the odometry quaternion values out of the message
  tf::Quaternion q(angle.orientation.x, angle.orientation.y,
                   angle.orientation.z, angle.orientation.w);

  // Turn the quaternion values into a matrix
  tf::Matrix3x3 m(q);

  // Extract the euler angles from the matrix
  m.getRPY(arr[0], arr[1], arr[2]);

  return arr;
}

/*----------------------------------------------------------------------------------*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "remote_teleop");

  RemoteTeleop remote_teleop_class;

  ros::spin();

  return 0;
}
