/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include "remote_teleop_robot_backend/PointClickNavActionGoal.h"

/*----------------------------------------------------------------------------------------------*/

visualization_msgs::Marker makeBox() {
  
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::ARROW;
  box_marker.scale.x = 1.0;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 1.0;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  
  return box_marker;
}

/*----------------------------------------------------------------------------------------------*/

visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg ) {
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox() );
  msg.controls.push_back( control );
  
  return msg.controls.back();
}

/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
//  ROS_INFO_STREAM(feedback->pose.position.x << "\t" << feedback->pose.position.y
//      << "\t" << feedback->pose.position.z /* << ") ORIENTATION (" <<feedback->pose.orientation.x << "\t" << feedback->pose.orientation.y << "\t" << feedback->pose.orientation.z << "\t" << feedback->pose.orientation.w*/);
      
  // Grab the odometry quaternion values out of the message
  tf::Quaternion q(
    feedback->pose.orientation.x,
    feedback->pose.orientation.y,
    feedback->pose.orientation.z,
    feedback->pose.orientation.w);
  
  // Turn the quaternion values into a matrix
  tf::Matrix3x3 m(q);
  
  tfScalar roll, pitch, yaw;
  
  // Extract the euler angles from the matrix
  m.getRPY(roll, pitch, yaw);
  
  ROS_INFO("%f\t%f\t%f", roll, pitch, yaw);
  
  ros::NodeHandle nh_;
  ros::Publisher pose_publisher_ = nh_.advertise<remote_teleop_robot_backend::PointClickNavActionGoal>("point_click_as/goal", 1);
  
  // TODO: publish message to the topic here and subscribe to the topic in the nav 
  
  remote_teleop_robot_backend::PointClickNavActionGoal msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.goal_id.stamp = ros::Time::now();
  msg.goal_id.id = "message";
  
  msg.goal.goal_pose.header.stamp = ros::Time::now();
  msg.goal.goal_pose.header.frame_id = "base_link";
  
  msg.goal.goal_pose.pose.position.x = feedback->pose.position.x;
  msg.goal.goal_pose.pose.position.y = feedback->pose.position.y;
  msg.goal.goal_pose.pose.position.z = feedback->pose.position.z;
  msg.goal.goal_pose.pose.orientation.x = feedback->pose.orientation.x;
  msg.goal.goal_pose.pose.orientation.y = feedback->pose.orientation.y;
  msg.goal.goal_pose.pose.orientation.z = feedback->pose.orientation.z;
  msg.goal.goal_pose.pose.orientation.w = feedback->pose.orientation.w;
  
  pose_publisher_.publish(msg);
  
}

/*----------------------------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  
  // initialize the node
  ros::init(argc, argv, "remote_teleop_interactive_marker");
  
//  ros::Publisher pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("point_click_as/goal", 1);

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("remote_teleop_interactive_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // create the box marker and the non-interactive control which contains the box
  makeBoxControl( int_marker );
  
  visualization_msgs::InteractiveMarkerControl control;
  
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
//  control.name = "rotate_x";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
//  control.name = "rotate_y";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//  int_marker.controls.push_back(control);
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
//  control.name = "move_z";
//  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//  int_marker.controls.push_back(control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
