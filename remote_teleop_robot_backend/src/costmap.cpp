#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv) {
  tf2_ros::Buffer tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);
}
