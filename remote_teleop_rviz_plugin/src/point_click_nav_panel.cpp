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

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QString>

#include <geometry_msgs/Twist.h>

#include "drive_widget.h"
#include "teleop_panel.h"
#include "point_click_nav_panel.h"

namespace rviz_plugin_tutorials
{

// TODO update the internal variables here
PointClickNavPanel::PointClickNavPanel( QWidget* parent )
  : rviz::Panel( parent )
  , position_x_( 0.0 )
  , position_y_( 0.0 )
  , position_z_( 0.0 )
  , orientation_x_( 0.0 )
  , orientation_y_( 0.0 )
  , orientation_z_( 0.0 )
  , orientation_w_( 0.0 )
  
{
  // TODO: Create the layout
  QVBoxLayout* topic_layout = new QVBoxLayout;
  
  // TODO: Figure out how to add text here
  topic_layout->addWidget( new QLabel( "Click on the screen to select\n\ra location." ));
  
  QPushButton* confirm_coords_button_ = new QPushButton(this);
  confirm_coords_button_->setText(tr("Confirm Coordinates"));
  topic_layout->addWidget( confirm_coords_button_ );
  
  setLayout( topic_layout );


  // TODO: Update the signal/slot connections
  connect()
  connect(confirm_coords_button_, SIGNAL(released()), this, SLOT(setNavGoal()));
  
  connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

}


void PointClickNavPanel::setNavGoal() {
  // TODO: set these values with the mouse event coordinates
  goal_coords_.pose.position.x = 0.0;
  goal_coords_.pose.position.y = 0.0;
  goal_coords_.pose.position.z = 0.0;
  
  // TODO: figure out how to get the goal orientation from a mouse click event
  goal_coords_.pose.orientation.x = 0.0;
  goal_coords_.pose.orientation.y = 0.0;
  goal_coords_.pose.orientation.z = 0.0;
  goal_coords_.pose.orientation.w = 0.0;
}


void PointClickNavPanel::sendNavGoal()
{
  // Make sure the publisher exists and ROS is not shutting down
  if( ros::ok() && nav_goal_publisher_ )
  {
    //TODO: get the correct message (nav goal action message)
    geometry_msgs::Twist msg;
    
    // TODO: update message fields here
    // this message will be a posestamped message
    
    // publish message
    nav_goal_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PointClickNavPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  // TODO: MAKE SURE THIS IS THE RIGHT TOPIC
  config.mapSetValue( "Topic", "point_click_nav_goal_as/goal" );
}

// TODO: CHECK THAT THIS IS DONE
// Load all configuration data for this panel from the given Config object.
void PointClickNavPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  turn_goal_publisher_ = nh_.advertise<remote_teleop_robot_backend::PointClickNavActionGoal>( "point_click_nav_goal_as/goal", 1 );
  Q_EMIT configChanged();
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(remote_teleop_rviz_plugin::PointClickNavPanel,rviz::Panel )
