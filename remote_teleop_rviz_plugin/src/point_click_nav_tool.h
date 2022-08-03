#ifndef POINTCLICKNAV_TOOL_H
#define POINTCLICKNAV_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>

#include <ros/ros.h>

//#include "rviz/default_plugin/tools/pose_tool.h"

#endif

namespace remote_teleop_rviz_plugin {

class Arrow;
class DisplayContext;
class StringProperty;

class PointClickNavTool : public PoseTool {
  
  Q_OBJECT
public:
  PointClickNavTool();
  ~PointClickNavTool() override {
  }
  
  void onInitialize() override;
  
protected:
  
  void onPoseSet(double x, double y, double theta) override;
  
private Q_SLOTS:
  void updateTopic();
  
private:
  ros::NodeHandle nh_;
  ros::Publisher nav_publisher_;
  //TODO:I don't think I need this?
  StringProperty* topic_property_;

};

// TODO: This needs to go in another file --> see example (pose_tool.h)
/*// TODO: might not need these*/
/*class Arrow;*/
/*class DisplayContext;*/

/*class PoseTool: public rviz::Tool {*/

/*public:*/

/*  PoseTool();*/
/*  ~PoseTool() override; // TODO: no idea what this means*/

/*  void onInitialize() override;*/
/*  */
/*  void activate() override;*/
/*  void deactivate() override;*/
/*  */
/*  int processMouseEvent(ViewportMouseEvent& event) override;*/
/*  */
/*protected:*/
/*  */
/*  virtual void onPoseSet(double x, double y, double theta) = 0;*/
/*  */
/*  Arrow* arrow_;*/
/*  */
/*  enum State {*/
/*    Position,*/
/*    Orientation*/
/*  };*/
/*  State state_;*/
/*  */
/*  Ogre::Vector3 pos_;*/
/*};*/

}

#endif
