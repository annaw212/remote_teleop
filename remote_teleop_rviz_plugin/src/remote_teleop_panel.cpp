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

#include <stdio.h>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>
#include <QValidator>

#include <remote_teleop_robot_backend/NudgeActionGoal.h>
#include <remote_teleop_robot_backend/NudgeActionResult.h>
#include <remote_teleop_robot_backend/NudgeResult.h>
#include <remote_teleop_robot_backend/PointClickNavActionGoal.h>
#include <remote_teleop_robot_backend/PointClickNavActionResult.h>
#include <remote_teleop_robot_backend/PointClickNavResult.h>
#include <remote_teleop_robot_backend/ResetMarkerActionGoal.h>
#include <remote_teleop_robot_backend/ResetMarkerActionResult.h>
#include <remote_teleop_robot_backend/ResetMarkerResult.h>
#include <remote_teleop_robot_backend/SpeedToggleActionGoal.h>
#include <remote_teleop_robot_backend/StopNavActionGoal.h>
#include <remote_teleop_robot_backend/StopNavActionResult.h>
#include <remote_teleop_robot_backend/StopNavResult.h>
#include <remote_teleop_robot_backend/TurnInPlaceActionGoal.h>
#include <remote_teleop_robot_backend/TurnInPlaceActionResult.h>
#include <remote_teleop_robot_backend/TurnInPlaceResult.h>
#include <remote_teleop_robot_backend/Velocity.h>

#include "remote_teleop_panel.h"

namespace remote_teleop_rviz_plugin {

// RemoteTeleopPanel class assignment
RemoteTeleopPanel::RemoteTeleopPanel(QWidget *parent)
    : rviz::Panel(parent), degrees_(30.0), turn_left_(true), lin_vel_(0.0),
      ang_vel_(0.0), nudge_dist_(0.0), nudge_fwd_(true) {

  // Create box for organizing all elements of the plugin
  // Vertical box layout so that everything stacks nicely
  QVBoxLayout *topic_layout = new QVBoxLayout;

  //  // Create a horizontal box for the degrees input box
  //  QHBoxLayout* degrees_layout = new QHBoxLayout;
  //  // Add a title for the input box
  //  degrees_layout->addWidget( new QLabel( "Degrees to turn:" ));
  //  // Create the input box
  //  degrees_topic_editor_ = new QLineEdit;
  //  // Set the input to only accept integers
  //  QValidator* validator = new QIntValidator(0, 100, this);
  //  degrees_topic_editor_->setValidator(validator);
  //  // Add the input box to the horizontal layout
  //  degrees_layout->addWidget( degrees_topic_editor_ );
  //  // Add the horizontal box to be the first item in the vertical box
  //  topic_layout->addLayout( degrees_layout );

  // Create buttons for determining which direction to turn
  // Create a horizontal box for both the buttons to go in so they
  // lay side-by-side

  QHBoxLayout *button_layout = new QHBoxLayout;

  // Create a button for turning left and add to the horizontal box
  QPushButton *turn_left_button_ = new QPushButton(this);
  turn_left_button_->setText(tr("Turn Left"));
  button_layout->addWidget(turn_left_button_);

  // Create a button for turning right and add to the horizontal box
  QPushButton *turn_right_button_ = new QPushButton(this);
  turn_right_button_->setText(tr("Turn Right"));
  button_layout->addWidget(turn_right_button_);

  QHBoxLayout *nudge_layout = new QHBoxLayout;

  // Create a button for turning left and add to the horizontal box
  QPushButton *nudge_fwd_button_ = new QPushButton(this);
  nudge_fwd_button_->setText(tr("Nudge Forward"));
  nudge_layout->addWidget(nudge_fwd_button_);

  // Create a button for turning right and add to the horizontal box
  QPushButton *nudge_bwd_button_ = new QPushButton(this);
  nudge_bwd_button_->setText(tr("Nudge Backward"));
  nudge_layout->addWidget(nudge_bwd_button_);

  // Create the layout for the point click navigation command
  QHBoxLayout *nav_layout = new QHBoxLayout;

  // Create a button for turning right and add to the horizontal box
  QPushButton *confirm_coords_ = new QPushButton(this);
  confirm_coords_->setText(tr("Confirm Coordinates"));
  nav_layout->addWidget(confirm_coords_);

  // Create a button for resetting the marker and add to the horizontal box
  QPushButton *reset_marker_ = new QPushButton(this);
  reset_marker_->setText(tr("Reset Marker"));
  nav_layout->addWidget(reset_marker_);

  // Create box layout for speed sliders
  QVBoxLayout *velocity_layout = new QVBoxLayout;

  velocity_layout->addWidget(new QLabel("Speed Toggles"));

  QHBoxLayout *lin_vel_layout = new QHBoxLayout;

  lin_vel_layout->addWidget(new QLabel("Linear Velocity:"));
  lin_vel_toggle_ = new QDoubleSpinBox(this);
  lin_vel_toggle_->setMaximum(1.5);
  lin_vel_toggle_->setMinimum(0.1);
  lin_vel_toggle_->setSuffix(" m/s");
  lin_vel_toggle_->setSingleStep(0.1);
  lin_vel_toggle_->setValue(0.5);
  lin_vel_toggle_->setDecimals(1);
  lin_vel_toggle_->setButtonSymbols(QAbstractSpinBox::PlusMinus);

  lin_vel_layout->addWidget(lin_vel_toggle_);

  lin_vel_layout->addWidget(new QLabel("\tAngular Velocity:"));
  ang_vel_toggle_ = new QDoubleSpinBox(this);
  ang_vel_toggle_->setMaximum(1.5);
  ang_vel_toggle_->setMinimum(0.1);
  ang_vel_toggle_->setSuffix(" m/s");
  ang_vel_toggle_->setSingleStep(0.1);
  ang_vel_toggle_->setValue(0.5);
  ang_vel_toggle_->setDecimals(1);
  ang_vel_toggle_->setButtonSymbols(QAbstractSpinBox::PlusMinus);

  lin_vel_layout->addWidget(ang_vel_toggle_);
  velocity_layout->addLayout(lin_vel_layout);

  // Add in virtual e-stop button
  QPushButton *stop_nav_button_ = new QPushButton(this);
  stop_nav_button_->setText(tr("STOP"));
  stop_nav_button_->setStyleSheet(
      "font:bold;background-color:red;font-size:36px;height:42px;width:100px");

  status_label_ = new QLabel("<b>Status: </b>");

  // Add the horizontal box to the vertical box layout
  topic_layout->addWidget(status_label_);
  topic_layout->addWidget(new QLabel("Point-and-Click Navigation"));
  topic_layout->addLayout(nav_layout);
  topic_layout->addWidget(new QLabel("Turn in Place"));
  topic_layout->addLayout(button_layout);
  topic_layout->addWidget(new QLabel("Nudge"));
  topic_layout->addLayout(nudge_layout);
  topic_layout->addLayout(velocity_layout);
  topic_layout->addWidget(stop_nav_button_);

  // Set the layout
  setLayout(topic_layout);

  // Make the slot connections
  // When the buttons are released (SIGNAL), it will trigger the calling of the
  // setTurnGoalX() function to run
  // NOTE: both the inputs to the signal and slot functions need to be the same
  connect(turn_left_button_, SIGNAL(released()), this, SLOT(setTurnGoalLeft()));
  connect(turn_right_button_, SIGNAL(released()), this,
          SLOT(setTurnGoalRight()));
  connect(confirm_coords_, SIGNAL(released()), this, SLOT(sendNavGoal()));
  connect(lin_vel_toggle_, SIGNAL(valueChanged(double)), this,
          SLOT(setVelGoal()));
  connect(ang_vel_toggle_, SIGNAL(valueChanged(double)), this,
          SLOT(setVelGoal()));
  connect(stop_nav_button_, SIGNAL(released()), this, SLOT(sendStopGoal()));
  connect(nudge_fwd_button_, SIGNAL(released()), this, SLOT(setNudgeGoalFwd()));
  connect(nudge_bwd_button_, SIGNAL(released()), this, SLOT(setNudgeGoalBwd()));
  connect(reset_marker_, SIGNAL(released()), this, SLOT(sendResetMarkerGoal()));
}

/*-----------------------------------------------------------------------------------*/

// setTurnGoalLeft() sets the degrees and direction variables and calls
// sendTurnGoal() for the new variable values to be published
void RemoteTeleopPanel::setTurnGoalLeft() {

  // Assign the value to send (this is a pre-determined static value)
  degrees_ = 30.0;
  // Set the turn_left_ internal variable
  turn_left_ = true;
  // Publish the message for the ROS node
  sendTurnGoal();
}

/*-----------------------------------------------------------------------------------*/

// setTurnGoalLeft() sets the degrees and direction variables and calls
// sendTurnGoal() for the new variable values to be published
void RemoteTeleopPanel::setTurnGoalRight() {

  // Assign the value to send (this is a pre-determined static value)
  degrees_ = 30.0;
  // Set the turn_left_ internal variable
  turn_left_ = false;
  // Publish the message for the ROS node
  sendTurnGoal();
}

/*-----------------------------------------------------------------------------------*/

// setVelGoal() allows the user to set the desired linear/angular
// velocity and calls sendVelGoal() for the new variable values to be published
void RemoteTeleopPanel::setVelGoal() {

  // Get the values from the sliders --> maybe something similar to fcn above
  lin_vel_ = lin_vel_toggle_->value();
  ang_vel_ = ang_vel_toggle_->value();
  // Publish the message for the ROS node
  sendVelGoal();
}

/*-----------------------------------------------------------------------------------*/

// setNudgeGoalFwd() sets the distance and direction variables and calls
// sendNudgeGoal() for the new variable values to be published
void RemoteTeleopPanel::setNudgeGoalFwd() {

  // Set the distance variable at 15cm
  nudge_dist_ = 0.15;
  // Set the direction to forward
  nudge_fwd_ = true;
  // Publish the message for the ROS node
  sendNudgeGoal();
}

/*-----------------------------------------------------------------------------------*/

// setNudgeGoalBwd() sets the distance and direction variables and calls
// sendNudgeGoal() for the new variable values to be published
void RemoteTeleopPanel::setNudgeGoalBwd() {

  // Set the distance variable at 15cm
  nudge_dist_ = 0.15; // 15cm
  // Set the direction to backwards
  nudge_fwd_ = false;
  // Publish the message for the ROS node
  sendNudgeGoal();
}

/*-----------------------------------------------------------------------------------*/

// Publish the degrees and direction if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RemoteTeleopPanel::sendTurnGoal() {

  // Make sure the publisher exists and ROS not shutting down
  if (ros::ok() && turn_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::TurnInPlaceActionGoal msg;
    // Set the message fields
    msg.goal.degrees = degrees_;
    msg.goal.turn_left = turn_left_;
    // Publish the message
    turn_goal_publisher_.publish(msg);
    // Update the status message
    status_label_->clear();
    if (turn_left_ == true) {
      status_label_->setText("<b>Status: Turning left.</b>");
    } else {
      status_label_->setText("<b>Status: Turning right.</b>");
    }
    
  }
}

/*-----------------------------------------------------------------------------------*/

// Publish the nav message if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RemoteTeleopPanel::sendNavGoal() {

  // Make sure the publisher exists and ROS not shutting down
  if (ros::ok() && nav_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::PointClickNavActionGoal msg;
    // Set the message fields
    msg.goal.coords_confirmed = true;
    // Publish the message
    nav_goal_publisher_.publish(msg);
    // Update the status message
    status_label_->clear();
    status_label_->setText("<b>Status: Starting navigation.</b>");
  }
}

/*-----------------------------------------------------------------------------------*/

// Publish the linear/angular velocity if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RemoteTeleopPanel::sendVelGoal() {

  // Make sure the publisher exists and ROS not shutting down
  if (ros::ok() && vel_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::SpeedToggleActionGoal msg;
    // Set the message fields
    msg.goal.lin_vel = lin_vel_;
    msg.goal.ang_vel = ang_vel_;
    // Publish the message
    vel_goal_publisher_.publish(msg);
  }
}

/*-----------------------------------------------------------------------------------*/

// Publish the stop goal if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RemoteTeleopPanel::sendStopGoal() {

  // Make sure the publisher exists and ROS not shutting down
  if (ros::ok() && stop_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::StopNavActionGoal msg;
    // Set the message fields
    msg.goal.stop = true;
    // Publish the message
    stop_goal_publisher_.publish(msg);
  }
}

/*-----------------------------------------------------------------------------------*/

// Publish the distance and direction if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void RemoteTeleopPanel::sendNudgeGoal() {

  // Make sure the publisher exists and ROS is not shutting down
  if (ros::ok() && nudge_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::NudgeActionGoal msg;
    // Set the message fields
    msg.goal.dist = nudge_dist_;
    msg.goal.fwd = nudge_fwd_;
    // Publish the message
    nudge_goal_publisher_.publish(msg);
    // Update the status message
    status_label_->clear();
    if (nudge_fwd_ == true) {
      status_label_->setText("<b>Status: Nudging forward.</b>");
    } else {
      status_label_->setText("<b>Status: Nudging backward.</b>");
    }
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::sendResetMarkerGoal() {

  // Make sure the publisher exists and ROS is not shutting down
  if (ros::ok() && reset_marker_goal_publisher_) {
    // Create a message of the desired type
    remote_teleop_robot_backend::ResetMarkerActionGoal msg;
    // Set the message fields
    msg.goal.reset_marker = true;
    // Publish the message
    reset_marker_goal_publisher_.publish(msg);
  }
}

/*-----------------------------------------------------------------------------------*/

// Upon the receipt of a Velocity message sent from the backend, update the
// internal linear/angular vleocity variables and update the Rviz velocity
// toggle values to reflect
// NOTE: This should only be called upon the startup of a new remote_teleop node
void RemoteTeleopPanel::velocityCallback(
    const remote_teleop_robot_backend::VelocityConstPtr &msg) {

  // Update the Rviz frontend velocity toggles to reflect the correct values
  lin_vel_toggle_->setValue(msg->lin_vel);
  ang_vel_toggle_->setValue(msg->ang_vel);

  // Update the internal variable values
  lin_vel_ = msg->lin_vel;
  ang_vel_ = msg->ang_vel;
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::pointClickResultCallback(
    const remote_teleop_robot_backend::PointClickNavActionResultConstPtr
        &result) {

  status_label_->clear();
  if (result->result.success == true) {
    status_label_->setText("<b>Status: Navigation completed.</b>");
  } else {
    if (result->result.obstacle == true) {
      status_label_->setText("<b>Status: Navigation failed. Obstacle detected.</b>");
    } else {
      status_label_->setText("<b>Status: Navigation failed. Time out error.</b>");
    }    
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::turnInPlaceResultCallback(
    const remote_teleop_robot_backend::TurnInPlaceActionResultConstPtr
        &result) {

  status_label_->clear();
  if (turn_left_ == true) {
    status_label_->setText("<b>Status: Left turn in place completed.</b>");
  } else {
    status_label_->setText("<b>Status: Right turn in place completed.</b>");
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::stopNavResultCallback(
    const remote_teleop_robot_backend::StopNavActionResultConstPtr &result) {

  status_label_->clear();
  status_label_->setText("<b>Status: Stopping navigation.</b>");
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::nudgeResultCallback(
    const remote_teleop_robot_backend::NudgeActionResultConstPtr &result) {

  status_label_->clear();
  if (nudge_fwd_ == true) {
    status_label_->setText("<b>Status: Forward nudge completed.</b>");
  } else {
    status_label_->setText("<b>Status: Backward nudge completed.</b>");
  }
}

/*-----------------------------------------------------------------------------------*/

void RemoteTeleopPanel::resetMarkerResultCallback(
    const remote_teleop_robot_backend::ResetMarkerActionResultConstPtr
        &result) {

  status_label_->clear();
  status_label_->setText(
      "<b>Status: Resetting marker to original position.</b>");
}

/*-----------------------------------------------------------------------------------*/

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void RemoteTeleopPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("Topic", "turn_in_place_as/goal");
}

/*-----------------------------------------------------------------------------------*/

// Load all configuration data for this panel from the given Config object.
void RemoteTeleopPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  // Initialize subscriber
  velocity_subscriber_ = nh_.subscribe(
      "/rt_initial_velocities", 1, &RemoteTeleopPanel::velocityCallback, this);
  nav_update_subscriber_ =
      nh_.subscribe("point_click_as/result", 1,
                    &RemoteTeleopPanel::pointClickResultCallback, this);
  turn_update_subscriber_ =
      nh_.subscribe("turn_in_place_as/result", 1,
                    &RemoteTeleopPanel::turnInPlaceResultCallback, this);
  stop_update_subscriber_ = nh_.subscribe(
      "stop_nav_as/result", 1, &RemoteTeleopPanel::stopNavResultCallback, this);
  nudge_update_subscriber_ = nh_.subscribe(
      "nudge_as/result", 1, &RemoteTeleopPanel::nudgeResultCallback, this);
  reset_marker_update_subscriber_ =
      nh_.subscribe("reset_marker_as/result", 1,
                    &RemoteTeleopPanel::resetMarkerResultCallback, this);

  // Initialize publishers
  turn_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::TurnInPlaceActionGoal>(
          "turn_in_place_as/goal", 1);
  nav_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::PointClickNavActionGoal>(
          "point_click_as/goal", 1);
  stop_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::StopNavActionGoal>(
          "stop_nav_as/goal", 1);
  nudge_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::NudgeActionGoal>(
          "nudge_as/goal", 1);
  vel_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::SpeedToggleActionGoal>(
          "speed_toggle_as/goal", 1);
  reset_marker_goal_publisher_ =
      nh_.advertise<remote_teleop_robot_backend::ResetMarkerActionGoal>(
          "reset_marker_as/goal", 1);
  Q_EMIT configChanged();
}

} // namespace remote_teleop_rviz_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(remote_teleop_rviz_plugin::RemoteTeleopPanel,
                       rviz::Panel)
