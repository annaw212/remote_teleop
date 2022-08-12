#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include "costmap.h"


using costmap_2d::LETHAL_OBSTACLE;

namespace remote_teleop_rviz_plugin {

Costmap::Costmap() {}

void Costmap::onInitialize() {

  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&Costmap::reconfigureCB, this, _1, _2);

  dsrv_->setCallback(cb);

}


void Costmap::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {

  enabled_ = config.enabled;

}


void Costmap::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {

  if(!enabled_) {
    return;
  }
  
  mark_x_ = origin_x + cos(origin_yaw);
  mark_y_ = origin_y + sin(origin_yaw);
  
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);

}


void Costmap::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {

  if(!enabled_) {
    return;
  }
  
  unsigned int mx, my;
  
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)) {
  
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  
  }

}

} //end namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(remote_teleop_rviz_plugin::Costmap, costmap_2d::Layer)
