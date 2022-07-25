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
#ifndef TURNINPLACE_PANEL_H
#define TURNINPLACE_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QLineEdit;

namespace remote_teleop_rviz_plugin
{

// Declare the TurnInPlacePanel class of type rviz panel
class TurnInPlacePanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:

  // QWidget subclass constructor
  TurnInPlacePanel( QWidget* parent = 0 );

  // Declare overrides of rviz::Panel functions for saving 
  // and loading from config file
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;


public Q_SLOTS:
  
  // Once the 'turn left' button has been pushed, this function is called to handle
  // the assignment of internal variables and call the sendTurnGoal function
  void setTurnGoalLeft();
  
  // Once the 'turn right' button has been pushed, this function is called to handle
  // the assignment of internal variables and call the sendTurnGoal function
  void setTurnGoalRight();


protected Q_SLOTS:

  // sendTurnGoal() checks the validity of the publisher and ROS,
  // creates a message of the desired type, assigns the fields of
  // that message to values, and then publishes the message
  void sendTurnGoal();


protected:

  // One-line text editor for entering the degrees to turn by in
  QLineEdit* degrees_topic_editor_;


  // The ROS publisher for the degrees and direction to turn in
  ros::Publisher turn_goal_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // Internal variables for storing the degrees and direction commands in
  float degrees_;
  bool turn_left_;

};

} 

#endif // TELEOP_PANEL_H
