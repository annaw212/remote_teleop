/* File: turn_in_place_server.cpp
 * Author: Anna Wong
 * Purpose:
 */

#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <interactive_markers/interactive_marker_server.h>
#include <iostream>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>

#include <remote_teleop_robot_backend/TurnInPlaceAction.h>
#include <remote_teleop_robot_backend/TurnInPlaceGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>

#include <remote_teleop_robot_backend/PointClickNavAction.h>
#include <remote_teleop_robot_backend/PointClickNavActionGoal.h>
#include <remote_teleop_robot_backend/PointClickNavGoal.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>

#include "remote_teleop_server.h"

/*-----------------------------------------------------------------------------------*/

// Define variables here
#define THRESHOLD 0.03
#define MIN_VEL 0.08
#define ANGLE_THRESHOLD 0.035

/*-----------------------------------------------------------------------------------*/

// CONSTRUCTOR: this will get called whenever an instance of this class is
// created
RemoteTeleop::RemoteTeleop()
    : turn_in_place_server_(
          nh_, "/turn_in_place_as",
          boost::bind(&RemoteTeleop::turn_in_place_callback, this, _1), false),
      point_click_server_(
          nh_, "/point_click_as",
          boost::bind(&RemoteTeleop::point_click_callback, this, _1), false),
      int_marker_server_("interactive_marker_server") {

  ROS_INFO("In class constructor of RemoteTeleop");

  // Initialize the messy stuff
  initializeIntMarkers();
  initializeSubscribers();
  initializePublishers();
  initializeActions();

  // Initialize the internal variables
  angle_ = 0.0;
  turn_left_ = true;
  lin_vel_ = 0.5;
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
  obstacle_detected_ = false;
  reading_costmap_ = false;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeSubscribers() {

  ROS_INFO("Initializing Subscribers");

  // Initialize the odometry subscriber
  odom_sub_ = nh_.subscribe("/odom", 1, &RemoteTeleop::odom_callback, this);

  // Initialize the costmap subscriber
  costmap_sub_ = nh_.subscribe("/rt_costmap_node/costmap/costmap", 1,
                               &RemoteTeleop::costmap_callback, this);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializePublishers() {

  ROS_INFO("Initializing Publishers");

  // Initialize the turn in place publisher
  turn_in_place_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  // Initialize the point and click publisher
  point_click_nav_publisher_ =
      nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeActions() {

  ROS_INFO("Starting action servers");

  // Start the turn in place action server
  turn_in_place_server_.start();

  // Start the point click action server
  point_click_server_.start();
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::initializeIntMarkers() {

  // Create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // Create the box marker and the non-interactive control which contains the
  // box
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

  // Add the interactive marker to our collection & tell the server to call
  // processIntMarkerFeedback() when feedback arrives for it
  int_marker_server_.insert(
      int_marker,
      boost::bind(&RemoteTeleop::processIntMarkerFeedback, this, _1));

  // 'commit' changes and send to all clients
  int_marker_server_.applyChanges();

  return;
}

/*-----------------------------------------------------------------------------------*/

visualization_msgs::Marker RemoteTeleop::makeIntMarker() {

  // Create a marker
  visualization_msgs::Marker marker;
  // Assign a type to the marker
  marker.type = visualization_msgs::Marker::ARROW;
  // Scale the marker
  marker.scale.x = 1.0;
  marker.scale.y = 0.45;
  marker.scale.z = 0.45;
  // Assign colors to the marker
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

/*-----------------------------------------------------------------------------------*/

visualization_msgs::InteractiveMarkerControl &
RemoteTeleop::makeIntMarkerControl(visualization_msgs::InteractiveMarker &msg) {

  // Create an interactive marker control
  visualization_msgs::InteractiveMarkerControl control;
  // Set the control variables
  control.always_visible = true;
  // Assign a marker to the control
  control.markers.push_back(makeIntMarker());
  // Assign the control to an interactive marker
  msg.controls.push_back(control);

  return msg.controls.back();
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::turn_in_place_callback(
    const remote_teleop_robot_backend::TurnInPlaceGoalConstPtr &goal) {

  ROS_INFO_STREAM("Lin vel: " << lin_vel_ << ", Ang vel: " << ang_vel_);

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

void RemoteTeleop::processIntMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  // Grab the incoming position info from the marker
  pos_x_ = feedback->pose.position.x;
  pos_y_ = feedback->pose.position.y;
  pos_z_ = feedback->pose.position.z;
  // Grab the incoming orientation info from the marker
  or_x_ = feedback->pose.orientation.x;
  or_y_ = feedback->pose.orientation.y;
  or_z_ = feedback->pose.orientation.z;
  or_w_ = feedback->pose.orientation.w;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::odom_callback(const nav_msgs::Odometry &msg) {

  // Grab the odometry position values out of the message
  x_ = msg.pose.pose.position.x;
  y_ = msg.pose.pose.position.y;
  z_ = msg.pose.pose.position.z;
  a_ = msg.pose.pose.orientation.x;
  b_ = msg.pose.pose.orientation.y;
  c_ = msg.pose.pose.orientation.z;
  d_ = msg.pose.pose.orientation.w;

  // Grab the odometry quaternion values out of the message
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

  // Turn the quaternion values into a matrix
  tf::Matrix3x3 m(q);

  // Extract the euler angles from the matrix
  m.getRPY(roll_, pitch_, yaw_);
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::costmap_callback(const nav_msgs::OccupancyGrid &grid) {
  // Lock the costmap into place so we can read its values
  //  if (reading_costmap_ == true) {
  //    costmap_mtx_.lock();
  //  } else {
  //    costmap_mtx_.unlock();
  //  }
  return;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::turn_in_place() {

  // Create message to be sent
  geometry_msgs::Twist command;

  // Set the unchanging fields
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;

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

  ROS_INFO_STREAM("Goal final angle: " << goal_yaw * 180 / M_PI
                                       << "\tAmt to turn: "
                                       << angle_ * 180 / M_PI);
  // Turn the robot until it reaches the desired angle
  while (abs(goal_yaw - yaw_) > THRESHOLD) {
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
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::point_click_callback(
    const remote_teleop_robot_backend::PointClickNavGoalConstPtr &msg) {

  // Store values of position and orientation in local variables so they don't
  // change during calculations
  float x = pos_x_; // in relation to the robot, which thinks it is at (0,0,0)
  float y = pos_y_;
  float z = pos_z_;
  float a = or_x_;
  float b = or_y_;
  float c = or_z_;
  float d = or_w_;

  // TODO: delete this
  tf::Quaternion quat(a_, b_, c_, d_);
  tf::Matrix3x3 mat(quat);
  tfScalar j, k, l;
  mat.getRPY(j, k, l);
  ROS_INFO_STREAM("\n");
  ROS_INFO_STREAM("Original orientation: (" << j << ", " << k << ", " << l
                                            << ")");

  // Declare local variables
  float travel_dist = 0.0;
  bool turn_left1 = true, turn_left2 = true;
  double theta1 = 0.0;
  tfScalar r, t, theta2 = 0.0;

  // Calculate the distance needed to travel
  travel_dist = sqrt(pow(x, 2) + pow(y, 2));

  // Calculate the angle needed to turn to face goal point
  theta1 = atan2(y, x);

  // Edge case checking
  if (abs(theta1) <= ANGLE_THRESHOLD) {
    // If we are only turning 1-3 degrees, just don't turn
    theta1 = 0;
  } else if (abs(M_PI - theta1) <= ANGLE_THRESHOLD) {
    // If we are turning to an angle within 1-3 degrees of a 180 turn, turn 180
    theta1 = M_PI;
  }

  // Determine validity of path
  float x1 = x_;
  float y1 = y_;
  float x2 = x1 + x;
  float y2 = y1 + y;
  float dx = abs(x2 - x1);
  float dy = abs(y1 - y1);

  if (dx > dy) {
    // Slope is less than 1
    obstacle_check(x1, y1, x2, y2, dx, dy, true);
  } else {
    // Slope is greater than 1
    obstacle_check(x1, y1, x2, y2, dx, dy, false);
  }

  if (obstacle_detected_ == true) {
    // Path was not clear -- reset variable and exit function
    obstacle_detected_ = false;
    return;
  }

  // Determine direction to turn, and turn to face goal location
  // The reason for having the navigation command inside this function instead
  // of having it be like the others, is because we need to change the value of
  // theta1 to be positive if we are turning right, but theta2 is based on
  // theta1's original value, so this is _one_ way to make sure theta1 can keep
  // its original value...
  if (theta1 < 0.0) {
    turn_left1 = false;
    navigate(theta1 * -1, turn_left1, 0.0, 0.0, 0.0);
  } else {
    turn_left1 = true;
    navigate(theta1, turn_left1, 0.0, 0.0, 0.0);
  }

  // NAVIGATE

  // Turn to face goal location - done in the previous chunk of code

  // TODO: delete this
  tf::Quaternion quat0(a_, b_, c_, d_);
  tf::Matrix3x3 mat0(quat0);
  mat0.getRPY(j, k, l);
  ROS_INFO_STREAM("Pre-Drive Orientation: (" << j * 180 / M_PI << ", "
                                             << k * 180 / M_PI << ", "
                                             << l * 180 / M_PI << ")");

  // Drive straight to goal location
  navigate(0.0, true, x, y, travel_dist);

  // TODO: delete this
  tf::Quaternion quat1(a_, b_, c_, d_);
  tf::Matrix3x3 mat1(quat1);
  mat1.getRPY(j, k, l);
  ROS_INFO_STREAM("Post-Drive Orientation: (" << j * 180 / M_PI << ", "
                                              << k * 180 / M_PI << ", "
                                              << l * 180 / M_PI << ")");

  // Calculate angle to turn by from goal to goal orientation
  tf::Quaternion q(a, b, c, d);
  tf::Matrix3x3 m(q);
  m.getRPY(r, t, theta2);

  ROS_INFO_STREAM("Goal Orientation: (" << r * 180 / M_PI << ", "
                                        << t * 180 / M_PI << ", "
                                        << theta2 * 180 / M_PI << ")");

  // Because theta2 is simply the angle to turn based on the original
  // orientation, we need to shift the degrees to turn appropriately
  theta2 = theta2 - theta1;

  ROS_INFO_STREAM("Goal angle to turn: " << theta2 * 180 / M_PI);

  // Make sure theta2 is within a known range
  while (theta2 > M_PI) {
    theta2 -= 2 * M_PI;
  }
  while (theta2 < -M_PI) {
    theta2 += 2 * M_PI;
  }

  // Determine direction to turn
  if (theta2 < 0.0) {
    // Turn right
    turn_left2 = false;
    ROS_INFO_STREAM(
        "Post processing goal angle to turn: " << theta2 * 180 / M_PI << "R");
    theta2 *= -1;
  } else {
    // Turn left
    turn_left2 = true;
    ROS_INFO_STREAM(
        "Post processing goal angle to turn: " << theta2 * 180 / M_PI << "L");
  }

  // Turn robot to goal orientation
  navigate(theta2, turn_left2, 0.0, 0.0, 0.0);

  // TODO: delete this
  tf::Quaternion quat2(a_, b_, c_, d_);
  tf::Matrix3x3 mat2(quat2);
  mat2.getRPY(j, k, l);
  ROS_INFO_STREAM("Final Orientation: (" << j * 180 / M_PI << ", "
                                         << k * 180 / M_PI << ", "
                                         << l * 180 / M_PI << ")");

  // Update the turn in place result and success fields
  point_click_result_.success = true;
  point_click_server_.setSucceeded(point_click_result_);

  // Snap the interactive marker back to (0,0,0)
  initializeIntMarkers();
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::navigate(float angle, bool turn_left, float x_dist,
                            float y_dist, float dist) {

  // Declare local variables
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
    // Determine goal coordinates
    goal_x = x_ + x_dist;
    goal_y = y_ + y_dist;
    start_x = x_;
    start_y = y_;

    // Drive straight
    while (dist - (sqrt(pow(x_ - start_x, 2) + pow(y_ - start_y, 2))) >
           THRESHOLD) {
      // Set the linear velocity
      command.linear.x = std::min(lin_vel_ * abs((goal_x - x_)),
                                  lin_vel_ * abs((goal_y - y_)));
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
    ROS_INFO("TURN IN PLACE");
    // Turn in place
    angle_ = angle;
    turn_left_ = turn_left;
    turn_in_place();

    return;
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleop::obstacle_check(float x1, float y1, float x2, float y2,
                                  float dx, float dy, bool smallSlope) {
  // Using Brensenham's line algorithm to produce the straight-line coordinates
  // between two points. Taking those points and checking their locations on the
  // obstacle grid to make sure there are no obstacles in the way of navigation.

  // Lock the costmap into place so it isn't updating while we are reading the
  // values
  // TODO: this might be faulty logic, so make sure to check this out...you know
  // why things are going wrong if they're going wrong. It's probably because of
  // this
  reading_costmap_ = true;

  // Brensenham's line algorithm
  int pk = 2 * dy - dx;
  for (int i = 0; i <= dx; i++) {

    // TODO: check the value of the occupancy grid against the path
    //    if (occupancy_grid[x1][x2] != 0.0) {
    //      obstacle_detected_ = true;
    //      return;
    //    }

    // checking either to decrement or increment the value
    // if we have to plot from (0,100) to (100,0)
    x1 < x2 ? x1++ : x1--;

    if (pk < 0) {
      // decision value will decide to plot
      // either  x1 or y1 in x's position
      if (smallSlope == true) {
        // putpixel(x1, y1, RED);
        pk = pk + 2 * dy;
      } else {
        //(y1,x1) is passed in xt
        // putpixel(y1, x1, YELLOW);
        pk = pk + 2 * dy;
      }
    } else {
      y1 < y2 ? y1++ : y1--;

      if (smallSlope == true) {

        // putpixel(x1, y1, RED);
      } else {
        //  putpixel(y1, x1, YELLOW);
      }
      pk = pk + 2 * dy - 2 * dx;
    }
  }

  // Allow the costmap mutex to be unlocked
  reading_costmap_ = false;
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
