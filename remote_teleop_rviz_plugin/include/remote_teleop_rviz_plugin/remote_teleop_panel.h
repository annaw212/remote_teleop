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
#ifndef REMOTETELEOP_PANEL_H
#define REMOTETELEOP_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QDoubleSpinBox>
#include <QLabel>
#include <remote_teleop_robot_backend/NudgeActionResult.h>
#include <remote_teleop_robot_backend/NudgeResult.h>
#include <remote_teleop_robot_backend/PointClickNavActionResult.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>
#include <remote_teleop_robot_backend/ResetMarkerActionResult.h>
#include <remote_teleop_robot_backend/ResetMarkerResult.h>
#include <remote_teleop_robot_backend/StopNavActionResult.h>
#include <remote_teleop_robot_backend/StopNavResult.h>
#include <remote_teleop_robot_backend/TurnInPlaceActionResult.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>
#include <remote_teleop_robot_backend/Velocity.h>

class QLineEdit;

namespace remote_teleop_rviz_plugin {

// Declare the RemoteTeleopPanel class of type rviz panel
class RemoteTeleopPanel : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructor
  RemoteTeleopPanel(QWidget *parent = 0);

  // Declare overrides of rviz::Panel functions for saving
  // and loading from config file
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

  // Once the 'turn left' button has been pushed, this function is called to
  // handle the assignment of internal variables and call the sendTurnGoal
  // function
  void setTurnGoalLeft();

  // Once the 'turn right' button has been pushed, this function is called to
  // handle the assignment of internal variables and call the sendTurnGoal
  // function
  void setTurnGoalRight();

  // Once one of the velocity input steppers has been toggled, this function
  // is called to handle the assignment of internal variables and call the
  // sendTurnGoal function
  void setVelGoal();

  // Once the 'nudge forward' button has been pushed, this function is called
  // to handle the assignment of internal variables and call the sendNudgeGoal
  // function
  void setNudgeGoalFwd();

  // Once the 'nudge backward' button has been pushed, this function is called
  // to handle the assignment of internal variables and call the sendNudgeGoal
  // function
  void setNudgeGoalBwd();

protected Q_SLOTS:

  // sendTurnGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendTurnGoal();

  // sendNavGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendNavGoal();

  // sendVelGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendVelGoal();

  // sendStopGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendStopGoal();

  // sendNudgeGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendNudgeGoal();

  // sendResetMarkerGoal() checks the validity of the publisher
  // and ROS, creates a message of the desired type, assigns the
  // fields of that message to values, and then publishes the message
  void sendResetMarkerGoal();

protected:
  // One-line text editor for entering the degrees to turn by in
  /*  QLineEdit* degrees_topic_editor_;*/
  QDoubleSpinBox *lin_vel_toggle_;
  QDoubleSpinBox *ang_vel_toggle_;
  QLabel *status_label_;

  // The ROS publisher for messages to be sent
  ros::Publisher turn_goal_publisher_;
  ros::Publisher nav_goal_publisher_;
  ros::Publisher vel_goal_publisher_;
  ros::Publisher stop_goal_publisher_;
  ros::Publisher nudge_goal_publisher_;
  ros::Publisher reset_marker_goal_publisher_;

  // The ROS subscriber for updating the initial velocities based on the backend
  // values
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber nav_update_subscriber_;
  ros::Subscriber turn_update_subscriber_;
  ros::Subscriber stop_update_subscriber_;
  ros::Subscriber nudge_update_subscriber_;
  ros::Subscriber reset_marker_update_subscriber_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // Internal variables for storing the degrees and direction commands in
  float degrees_;    // Turn in place
  bool turn_left_;   // Turn in place
  float lin_vel_;    // Velocity
  float ang_vel_;    // Velocity
  float nudge_dist_; // Nudge
  bool nudge_fwd_;   // Nudge

  // velocityCallback() is subscribed to an incoming Velocity message that is
  // sent by the backend upon the startup of the remote_teleop node and receives
  // the current linear and angular velocity values that are set on the backend
  // so the Rviz frontend can update their currently shown values accordingly
  void
  velocityCallback(const remote_teleop_robot_backend::VelocityConstPtr &msg);

  // pointClickResultCallback() is subscribed to the point_click_as/result topic
  // and receives the incoming updates. It updates the status bar accordingly.
  void pointClickResultCallback(
      const remote_teleop_robot_backend::PointClickNavActionResultConstPtr
          &result);

  // turnInPlaceResultCallback() is subscribed to the turn_in_place_as/result
  // topic and receives the incoming updates. It updates the status bar
  // accordingly.
  void turnInPlaceResultCallback(
      const remote_teleop_robot_backend::TurnInPlaceActionResultConstPtr
          &result);

  // stopNavResultCallback() is subscribed to the stop_nav_as/result
  // and receives the incoming updates. It updates the status bar accordingly.
  void stopNavResultCallback(
      const remote_teleop_robot_backend::StopNavActionResultConstPtr &result);

  // nudgeResultCallback() is subscribed to the nudge_as/result topic
  // and receives the incoming updates. It updates the status bar accordingly.
  void nudgeResultCallback(
      const remote_teleop_robot_backend::NudgeActionResultConstPtr &result);

  // resetMarkerResultCallback() is subscribed to the reset_marker_as/result
  // topic and receives the incoming updates. It updates the status bar
  // accordingly.
  void resetMarkerResultCallback(
      const remote_teleop_robot_backend::ResetMarkerActionResultConstPtr
          &result);
};

} // namespace remote_teleop_rviz_plugin

#endif // REMOTETELEOP_PANEL_H
