#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include "costmap.h"


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace my_costmap {

Costmap::Costmap() {}

void Costmap::onInitialize() {

  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&Costmap::reconfigureCB, this, _1, _2);

  dsrv_->setCallback(cb);

}


void Costmap::matchSize() {

  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());

}

void Costmap::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {

  enabled_ = config.enabled;

}


void Costmap::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) {

  if(!enabled_) {
    return;
  }
  
  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);
  
  if(worldToMap(mark_x, mark_y, mx, my)) {
    setCost(mx, my, LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);

}


void Costmap::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {

  if (!enabled_) {
    return;
  }
  
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }

}

} //end namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_costmap::Costmap, costmap_2d::Layer)
