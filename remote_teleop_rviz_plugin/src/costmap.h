#ifndef COSTMAP_H
#define COSTMAP_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace remote_teleop_rviz_plugin
{

class Costmap : public costmap_2d::Layer {

public:

  Costmap();
  
  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  
private:
  
  ros::NodeHandle nh_;

  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  
  double mark_x_, mark_y_;
  
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

};
}

#endif
