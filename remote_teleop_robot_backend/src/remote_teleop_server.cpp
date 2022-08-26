/* File: remote_teleop_server.cpp
 * Author: Anna Wong
 * Purpose:
 */
#include <ros/ros.h>

#include <cmath>
#include <mutex>

#include <tf/tf.h>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

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

#include "remote_teleop_server.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <typeinfo>
/*-----------------------------------------------------------------------------------*/

// Define variables here
#define THRESHOLD 0.03
#define MIN_VEL 0.08
#define ANGLE_THRESHOLD 0.035
#define INIT_LIN_VEL 0.5
#define INIT_ANG_VEL 1.0

/*-----------------------------------------------------------------------------------*/

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
          boost::bind(&RemoteTeleop::speedToggleCallback, this, _1), false) {

  ROS_INFO("In class constructor of RemoteTeleop");

  // Initialize the messy stuff
  ROS_INFO("Initializing Markers");
  initializeIntMarkers("a");
  initializeSubscribers();
  initializePublishers();
  initializeActions();

  // Initialize the internal variables
  angle_ = 0.0;
  turn_left_ = true;
  lin_vel_ = INIT_LIN_VEL;
  ang_vel_ = INIT_ANG_VEL;
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
  point_click_running_ = false;
  obstacle_detected_ = false;
  stop_ = false;

  // Send the initial velocities to rviz
  remote_teleop_robot_backend::Velocity init_vel_msg;
  init_vel_msg.lin_vel = INIT_LIN_VEL;
  init_vel_msg.ang_vel = INIT_ANG_VEL;

  velocity_publisher_.publish(init_vel_msg);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeSubscribers() {

  ROS_INFO("Initializing Subscribers");

  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("/odom", 1, &RemoteTeleop::odomCallback, this);

  // Initialize the costmap subscriber
  costmap_sub_ = nh_.subscribe("/rt_costmap_node/costmap/costmap", 1,
                               &RemoteTeleop::costmapCallback, this);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializePublishers() {

  ROS_INFO("Initializing Publishers");

  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the point and click publisher
  point_click_nav_publisher_ =
      nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the stop publisher
  stop_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the marker publisher
  marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 5);

  // Initialize the occupancy grid debug publisher
  occupancy_grid_debug_publisher_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_debug", 5);

  // Initialize the nudge publisher
  nudge_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the velocity publisher
  velocity_publisher_ = nh_.advertise<remote_teleop_robot_backend::Velocity>(
      "rt_initial_velocities", 1);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeActions() {

  ROS_INFO("Starting Action Servers");

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
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeIntMarkers(std::string type) {

  //  ROS_INFO("Initializing Markers");

  // Create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // Create the box marker and the non-interactive control containing the box
  makeIntMarkerControl(int_marker, type);

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

/*-----------------------------------------------------------------------------------*/

visualization_msgs::Marker RemoteTeleop::makeIntMarker(std::string type) {

  // Create a marker
  visualization_msgs::Marker marker;
  // Assign a type to the marker
  if (type == "a") {
    marker_.type = visualization_msgs::Marker::ARROW;
    // Scale the marker
    marker.scale.x = 1.0;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    // Assign colors to the marker
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

  } else {
    marker_.action = visualization_msgs::Marker::MODIFY;
    // Scale the marker
    marker.scale.x = 1.0;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    // Assign colors to the marker
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.0;
  }

  return marker;
}

/*-----------------------------------------------------------------------------------*/

visualization_msgs::InteractiveMarkerControl &
RemoteTeleop::makeIntMarkerControl(visualization_msgs::InteractiveMarker &msg,
                                   std::string type) {

  // Create an interactive marker control
  visualization_msgs::InteractiveMarkerControl control;
  // Set the control variables
  control.always_visible = true;
  // Assign a marker to the control
  control.markers.push_back(makeIntMarker("a"));
  // Assign the control to an interactive marker
  msg.controls.push_back(control);

  return msg.controls.back();
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::turnInPlaceCallback(
    const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr &goal) {

  // TODO: gray out rviz plugin buttons when turn is being executed

  // Set a variable to "claim" the drivers
  turn_in_place_running_ = true;

  // TODO: check if point_and_click is running

  // Get inputs from Rviz and store them in variables
  angle_ = goal->degrees;
  turn_left_ = goal->turn_left;

  // Convert from degrees to radians
  angle_ = angle_ * M_PI / 180;

  initializeIntMarkers("d");

  // Tell robot to turn the desired angle
  turnInPlace();

  initializeIntMarkers("a");

  // Update the turn in place result and success fields
  turn_in_place_result_.success = true;
  turn_in_place_server_.setSucceeded(turn_in_place_result_);

  turn_in_place_running_ = false;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::processIntMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  // TODO: PHASE THESE OUT
  // Grab the incoming position info from the marker
  pos_x_ = feedback->pose.position.x;
  pos_y_ = feedback->pose.position.y;
  pos_z_ = feedback->pose.position.z;
  // Grab the incoming orientation info from the marker
  or_x_ = feedback->pose.orientation.x;
  or_y_ = feedback->pose.orientation.y;
  or_z_ = feedback->pose.orientation.z;
  or_w_ = feedback->pose.orientation.w;

  nav_goal_pose_ = feedback->pose;

  nav_goal_frame_ = feedback->header.frame_id;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::odomCallback(const nav_msgs::Odometry &msg) {

  // TODO: PHASE THESE OUT
  // Grab the odometry position and orientation values out of the message
  x_ = msg.pose.pose.position.x;
  y_ = msg.pose.pose.position.y;
  z_ = msg.pose.pose.position.z;
  a_ = msg.pose.pose.orientation.x;
  b_ = msg.pose.pose.orientation.y;
  c_ = msg.pose.pose.orientation.z;
  d_ = msg.pose.pose.orientation.w;

  current_odom_pose_ = msg.pose.pose;

  //  ROS_INFO_STREAM(msg.header.frame_id);

  // Grab the odometry quaternion values out of the message
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

  // Turn the quaternion values into a matrix
  tf::Matrix3x3 m(q);

  // Extract the euler angles from the matrix
  m.getRPY(roll_, pitch_, yaw_);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::costmapCallback(const nav_msgs::OccupancyGrid &grid) {
  // Store the values of the occupancy grid in a variable for future reference
  occupancy_grid_ = grid;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::nudgeCallback(
    const remote_teleop_robot_backend::NudgeGoalConstPtr &goal) {

  nudge_dist_ = goal->dist;
  nudge_fwd_ = goal->fwd;

  initializeIntMarkers("d");

  if (nudge_fwd_ == true && !stop_) {
    nudge(nudge_dist_, 0.0, nudge_dist_);
  } else if (nudge_fwd_ == false && !stop_) {
    nudge(-1 * nudge_dist_, 0.0, -1 * nudge_dist_);
  }

  if (stop_) {
    stopMovement();
    stop_ = false;
  }

  nudge_result_.success = true;
  nudge_server_.setSucceeded(nudge_result_);

  initializeIntMarkers("a");
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::speedToggleCallback(
    const remote_teleop_robot_backend::SpeedToggleGoalConstPtr &goal) {

  // Update the velocity variables
  lin_vel_ = goal->lin_vel;
  ang_vel_ = goal->ang_vel;

  velocity_result_.success = true;
  velocity_server_.setSucceeded(velocity_result_);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::turnInPlace() {

  turn_in_place_running_ = true;

  // Create message to be sent
  geometry_msgs::Twist command;

  // Set the unchanging fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  float goal_yaw = 0.0;

  if (turn_left_ == false) {
    // TURNING RIGHT
    goal_yaw = yaw_ - angle_;
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
    goal_yaw = yaw_ + angle_;
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
  while (abs(goal_yaw - yaw_) > THRESHOLD && !stop_) {

    // Set the turn rate
    command.angular.z = ang_vel_ * (goal_yaw - yaw_);

    // Ensure the robot will be turning in the correct direction
    if (turn_left_ == true && command.angular.z < 0.0) {
      command.angular.z *= -1;
    } else if (turn_left_ == false && command.angular.z > 0.0) {
      command.angular.z *= -1;
    }

    // Ensure the robot will never be turning at a speed greater than the
    // desired angular velocity
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

  turn_in_place_running_ = false;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::pointClickCallback(
    const remote_teleop_robot_backend::PointClickNavGoalConstPtr &msg) {

  point_click_running_ = true;

  // Store values of position and orientation in local variables so they don't
  // change during calculations
  geometry_msgs::Pose goal_pose;
  goal_pose = nav_goal_pose_;
  // TODO: phase these out
  float x = pos_x_; // in relation to the robot, which thinks it is at (0,0,0)
  float y = pos_y_;
  float z = pos_z_;
  float a = or_x_;
  float b = or_y_;
  float c = or_z_;
  float d = or_w_;

  // put that into base_link as a sanity check
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped nav_goal_frame_to_base_link;
  nav_goal_frame_to_base_link = tf_buffer.lookupTransform(
      "base_link", nav_goal_frame_, ros::Time(0), ros::Duration(1.0));

  // Input the point you want to transform and indicate we want to just
  // overwrite that object with the transformed point values
  // TODO: make sure these values work
  geometry_msgs::PoseStamped base_link_goal;
  base_link_goal.pose.position.x = goal_pose.position.x;
  base_link_goal.pose.position.y = goal_pose.position.y;
  base_link_goal.pose.position.z = goal_pose.position.z;
  base_link_goal.pose.orientation.w = goal_pose.orientation.w;
  base_link_goal.pose.orientation.x = goal_pose.orientation.x;
  base_link_goal.pose.orientation.y = goal_pose.orientation.y;
  base_link_goal.pose.orientation.z = c = goal_pose.orientation.z;
  tf2::doTransform(base_link_goal, base_link_goal, nav_goal_frame_to_base_link);

  // Declare local variables
  float travel_dist = 0.0;
  bool turn_left1 = true, turn_left2 = true;
  double theta1 = 0.0;
  tfScalar r, t, theta2 = 0.0;

  // Calculate the angle needed to turn to face goal point
  // TODO: figure out a better way to do this
  // TODO: Change these variables from x_ to odom x or whatever
  if (abs(x_ - x) <= 0.001 && abs(y_ - y) <= 0.001) {
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

  // Determine validity of path

  // TODO: don't need to create unnecessary variables
  // TODO: rename these variables
  // TODO: move the costmap translation somewhere else :)
  float x1 = current_odom_pose_.position.x -
             occupancy_grid_.info.origin.position.x; // Robot's current location
  float y1 = y_ - occupancy_grid_.info.origin.position.y;
  float x2 = goal_pose.position.x; // x1 + x, Robot's goal location
  float y2 = goal_pose.position.y;
  float dx = abs(x2 - x1);
  float dy = abs(y2 - y1);

  // Transform goal to odom
  geometry_msgs::PoseStamped pose;
  pose = transformGoalToOdom(x2, y2);

  // Set the goal coordinates on the costmap grid
  // TODO: make these their own variables too
  int goal_x;
  int goal_y;
  x2 = ceil(pose.pose.position.x / occupancy_grid_.info.resolution);
  y2 = ceil(pose.pose.position.y / occupancy_grid_.info.resolution);

  //  // Set the current coordinates on the costmap grid
  // TODO: make these their own variables --> be more descriptive --> these are
  // the costmap indices, not floats
  int curr_x;
  int curr_y;
  x1 = ceil(x1 / occupancy_grid_.info.resolution);
  y1 = ceil(y1 / occupancy_grid_.info.resolution);

  // Need to recalculate the dx/dy because they are now outdated
  dx = abs(x2 - x1);
  dy = abs(y2 - y1);
  if (abs(x_ - x) >= 0.001 && abs(y_ - y) >= 0.001) {
    if (dx > dy) {
      // Slope is less than 1
      //      ROS_INFO("HERE1");
      obstacleCheck(x1, y1, x2, y2, dx, dy, true);
    } else {
      // Slope is greater than 1
      //      ROS_INFO("HERE2");
      obstacleCheck(x1, y1, x2, y2, dx, dy, false);
    }
  }

  if (obstacle_detected_ == true) {
    // Path was not clear -- reset variable and exit function
    obstacle_detected_ = false;
    // Update the turn in place result and success fields
    point_click_result_.success = true;
    point_click_server_.setSucceeded(point_click_result_);
    // Snap the interactive marker back to (0,0,0)
    initializeIntMarkers("a");
    return;
  }

  ROS_INFO("SAFE TO NAVIGATE");

  // Calculate the distance needed to travel
  // x_ : base_link position in odom
  // x : marker goal position (most likely in odom, but in nav_goal_frame_ for
  // sure)
  travel_dist = sqrt(pow(abs(x - x_), 2) + pow(abs(y - y_), 2));

  // TODO: Delete the interactive marker so it's not confusing during navigation
  initializeIntMarkers("d");

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
      navigate(theta1 * -1, turn_left1, 0.0, 0.0, 0.0, x2, y2, dx, dy);
    } else {
      turn_left1 = true;
      navigate(theta1, turn_left1, 0.0, 0.0, 0.0, x2, y2, dx, dy);
    }

    // Drive straight to goal location
    navigate(0.0, true, x, y, travel_dist, x2, y2, dx, dy);

    // Calculate angle to turn by from goal to goal orientation
    tf::Quaternion q(
        base_link_goal.pose.orientation.x, base_link_goal.pose.orientation.y,
        base_link_goal.pose.orientation.z, base_link_goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(r, t, theta2);

    // Because theta2 is simply the angle to turn based on the original
    // orientation, we need to shift the degrees to turn appropriately
    theta2 = theta2 - theta1;

    // Make sure theta2 is within a known range
    //    while (theta2 > M_PI) {
    //      theta2 -= 2 * M_PI;
    //    }
    //    while (theta2 < -M_PI) {
    //      theta2 += 2 * M_PI;
    //    }

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
    navigate(theta2, turn_left2, 0.0, 0.0, 0.0, x2, y2, dx, dy);

  } else {
    stopMovement();
  }

  // Update the turn in place result and success fields
  point_click_result_.success = true;
  point_click_server_.setSucceeded(point_click_result_);

  // Snap the interactive marker back to (0,0,0)
  initializeIntMarkers("a");

  point_click_running_ = false;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::navigate(float angle, bool turn_left, float x_dist,
                            float y_dist, float dist, float x2, float y2,
                            float dx, float dy) {

  if (stop_) {
    stopMovement();
    return;
  }
  // Declare local variables
  float goal_x, goal_y, start_x, start_y;

  // init frame to base link

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
    goal_x = x_ + x_dist;
    goal_y = y_ + y_dist;
    start_x = x_;
    start_y = y_;
    // Drive straight
    while (abs(dist) - (sqrt(pow(x_ - start_x, 2) + pow(y_ - start_y, 2))) >
               THRESHOLD &&
           !stop_) {

      // Check for obstacles
      float x1 = ceil(x_ - occupancy_grid_.info.origin.position.x /
                               occupancy_grid_.info.resolution);
      float y1 = ceil(y_ - occupancy_grid_.info.origin.position.y /
                               occupancy_grid_.info.resolution);
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
        initializeIntMarkers("a");
        return;
      }

      // Set the linear velocity
      command.linear.x = std::min(lin_vel_ * abs((goal_x - x_)),
                                  lin_vel_ * abs((goal_y - y_)));

      if (command.linear.x > lin_vel_) {
        command.linear.x = lin_vel_;
      } else if (command.linear.x < MIN_VEL) {
        command.linear.x = MIN_VEL;
      }
      if (dist < 0.0) {
        command.linear.x *= -1;
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
    angle_ = angle;
    turn_left_ = turn_left;
    turnInPlace();

    return;
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::stopNavCallback(
    const remote_teleop_robot_backend::StopNavGoalConstPtr &goal) {
  stop_ = goal->stop;

  initializeIntMarkers("d");
  while (turn_in_place_running_ || point_click_running_) {
    // waiting
  }
  stop_ = false;
  initializeIntMarkers("a");
  stop_nav_result_.success = true;
  stop_action_server_.setSucceeded(stop_nav_result_);
}

/*-----------------------------------------------------------------------------------*/

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

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::obstacleCheck(float x1, float y1, float x2, float y2,
                                 float dx, float dy, bool smallSlope) {
  // Using Brensenham's line algorithm to produce the straight-line coordinates
  // between two points. Taking those points and checking their locations on the
  // obstacle grid to make sure there are no obstacles in the way of navigation.

  // Create all the necessary variables
  int w = occupancy_grid_.info.width;
  int idx;

  // Brensenham's line algorithm -- geeks4geeks
  int pk = 2 * dy - dx;

  for (int h = 0; h <= dx; h++) {

    // Find the index of the point we are currently looking at
    idx = ceil(y1 * w + x1);

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
/*-----------------------------------------------------------------------------------*/

geometry_msgs::PoseStamped RemoteTeleop::transformGoalToOdom(float goal_x,
                                                             float goal_y) {
  // Create all the necessary variables
  geometry_msgs::PoseStamped robot_pose;

  // Set the robot_pose values --> these are the values of our goal since that
  // is the point that needs to be converted from base link to odom frame
  robot_pose.pose.position.x = goal_x;
  robot_pose.pose.position.y = goal_y;
  robot_pose.pose.position.z = 0;
  robot_pose.pose.orientation.w = 1.0;

  // Create the objects needed for the transform
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped nav_goal_frame_to_odom;

  // Lookup the transform from the initial frame to odom and store in variable
  nav_goal_frame_to_odom = tf_buffer.lookupTransform(
      "odom", nav_goal_frame_, ros::Time(0), ros::Duration(1.0));

  // Input the point you want to transform and indicate we want to just
  // overwrite that object with the transformed point values
  tf2::doTransform(robot_pose, robot_pose,
                   nav_goal_frame_to_odom); // robot_pose is the PoseStamped I
                                            // want to transform

  // The output value will be slightly offset, so we need to translate it to the
  // costmap center based on the odom offset
  // TODO: move the costmap translation out of this function
  robot_pose.pose.position.x -= occupancy_grid_.info.origin.position.x;
  robot_pose.pose.position.y -= occupancy_grid_.info.origin.position.y;

  return robot_pose;
}
/*-----------------------------------------------------------------------------------*/

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
  float goal_x = x_ + x_dist;
  float goal_y = y_ + y_dist;
  float start_x = x_;
  float start_y = y_;
  // Drive straight
  while (abs(dist) - (sqrt(pow(x_ - start_x, 2) + pow(y_ - start_y, 2))) >
             THRESHOLD &&
         !stop_) {

    // Set the linear velocity
    command.linear.x =
        std::min(lin_vel_ * abs((goal_x - x_)), lin_vel_ * abs((goal_y - y_)));

    if (command.linear.x > lin_vel_) {
      command.linear.x = lin_vel_;
    } else if (command.linear.x < MIN_VEL) {
      command.linear.x = MIN_VEL;
    }
    if (dist < 0.0) {
      command.linear.x *= -1;
    }

    // Publish the command
    point_click_nav_publisher_.publish(command);
  }
  // Stop the robot from moving
  command.linear.x = 0.0;
  point_click_nav_publisher_.publish(command);
  return;
}
/*-----------------------------------------------------------------------------------*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "remote_teleop");

  ROS_INFO("Main: instantiating an object of type RemoteTeleop");

  RemoteTeleop remote_teleop_class;

  ROS_INFO("Main: going into spin");

  ros::spin();

  return 0;
}
