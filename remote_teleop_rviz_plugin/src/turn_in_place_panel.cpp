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

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QIntValidator>
#include <QValidator>
#include <QString>

#include <remote_teleop_robot_backend/TurnInPlaceActionGoal.h>

#include "turn_in_place_panel.h"

namespace remote_teleop_rviz_plugin
{

// TurnInPlacePanel class assignment
TurnInPlacePanel::TurnInPlacePanel( QWidget* parent )
  : rviz::Panel( parent )
  , degrees_( 0 )
  , turn_left_( true )
{
  
  // Create box for organizing all elements of the plugin
  // Vertical box layout so that everything stacks nicely
  QVBoxLayout* topic_layout = new QVBoxLayout;
  
  // Create a horizontal box for the degrees input box
  QHBoxLayout* degrees_layout = new QHBoxLayout;
  // Add a title for the input box
  degrees_layout->addWidget( new QLabel( "Degrees to turn:" ));
  // Create the input box
  degrees_topic_editor_ = new QLineEdit;
  // Set the input to only accept integers
  QValidator* validator = new QIntValidator(0, 100, this);
  degrees_topic_editor_->setValidator(validator);
  // Add the input box to the horizontal layout
  degrees_layout->addWidget( degrees_topic_editor_ );
  // Add the horizontal box to be the first item in the vertical box
  topic_layout->addLayout( degrees_layout );

  // Create buttons for determining which direction to turn
  
  // Create a horizontal box for both the buttons to go in so they
  // lay side-by-side
  QHBoxLayout* button_layout = new QHBoxLayout;
  
  // Create a button for turning left and add to the horizontal box
  QPushButton* turn_left_button_ = new QPushButton(this);
  turn_left_button_->setText(tr("Turn Left"));
  button_layout->addWidget( turn_left_button_ );
  
  // Create a button for turning right and add to the horizontal box
  QPushButton* turn_right_button_ = new QPushButton(this);
  turn_right_button_->setText(tr("Turn Right"));
  button_layout->addWidget( turn_right_button_ );
  
  // Add the horizontal box to the vertical box layout
  topic_layout->addLayout( button_layout );

  // Set the layout
  setLayout( topic_layout );

  
  // Make the slot connections
  // When the buttons are released (SIGNAL), it will trigger the calling of the
  // setTurnGoalX() function to run
  // NOTE: both the inputs to the signal and slot functions need to be the same
  connect(turn_left_button_, SIGNAL(released()), this, SLOT(setTurnGoalLeft()));
  connect(turn_right_button_, SIGNAL(released()), this, SLOT(setTurnGoalRight()));
}

// setTurnGoalLeft() sets the degrees and direction variables and calls
// sendTurnGoal() for the new variable values to be published
void TurnInPlacePanel::setTurnGoalLeft()
{

  // Get the input from the degrees input box
  QString str = degrees_topic_editor_->text();
  
  // Convert the variable from string to float
  // and set the internal variable value
  // NOTE: if the string is empty, the variable just becomes 0.0
  degrees_ = str.toFloat();
  
  // Set the turn_left_ internal variable
  turn_left_ = true;
  
  // Publish the message for the ROS node
  sendTurnGoal();
}

// setTurnGoalLeft() sets the degrees and direction variables and calls
// sendTurnGoal() for the new variable values to be published
void TurnInPlacePanel::setTurnGoalRight()
{
  // Get the input from the degrees input box
  QString str = degrees_topic_editor_->text();
  
  // Convert the variable from string to float
  // and set the internal variable value
  // NOTE: if the string is empty, the variable just becomes 0.0
  degrees_ = str.toFloat();
  
  // Set the turn_left_ internal variable
  turn_left_ = false;
  
  // Publish the message for the ROS node
  sendTurnGoal();
}


// Publish the degrees and direction if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void TurnInPlacePanel::sendTurnGoal()
{
  // Make sure the publisher exists and ROS not shutting down
  if( ros::ok() && turn_goal_publisher_ )
  {
    // Create a message of the desired type
    remote_teleop_robot_backend::TurnInPlaceActionGoal msg;
    
    // Set the message fields
    msg.goal.degrees = degrees_;
    msg.goal.turn_left = turn_left_;
    
    // Publish the message
    turn_goal_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TurnInPlacePanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", "turn_in_place_as/goal" );
}

// Load all configuration data for this panel from the given Config object.
void TurnInPlacePanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  turn_goal_publisher_ = nh_.advertise<remote_teleop_robot_backend::TurnInPlaceActionGoal>( "turn_in_place_as/goal", 1 );
  Q_EMIT configChanged();
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(remote_teleop_rviz_plugin::TurnInPlacePanel,rviz::Panel )
