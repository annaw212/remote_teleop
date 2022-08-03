#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helper/arrow.h> // DON'T NEED THIS PROBABLY - to be replaced with plant flag marker thing
#include <rviz/properties/string_property.h>

#include "point_click_nav_tool.h"

namespace remote_teleop_rviz_plugin {

PointClickNavTool::PointClickNavTool {
  shortcut_key_ = 'g';
  
  topic_property_ = new StringProperty("Topic", "goal", "The topic on which to publish navigation goals.", getPropertyContainer(), SLOT(updateTopic()), this);
  
}

void PointClickNavTool::onInitialize() {

  PoseTool::onInitialize();
  arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);
  setName("Point-and-Click Nav")
  updateTopic();
  
}

void PointClickNavTool::updateTopic() {

  try {
    // TODO: Update htis with the actual topic name
    nav_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
  }
  
  catch (const ros::Exception& e) {
    ROS_ERROR_STREAM_NAMED("PointClickNavTool", e.what());
  }

}

void PointClickNavGoal::onPoseSet(double x, double y, double theta) {

  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::PoseStamped goal;
  goal.pose.orientation = tf2::toMsg(quat);
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = ros::Time::now();
  ROS_INFO("Setting goal: Frame: %s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %3f\n", fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  nav_publisher_.publish(goal);

}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(remote_teleop_rviz_plugin::PointClickNavTool,rviz::Tool )
