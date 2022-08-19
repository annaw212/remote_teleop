import rospy
from nav_msgs.msg import OccupancyGrid
import math

def callback(msg):
  # republish occupancy grid or something
  
  goal_x = 1.0
  goal_y = 1.0
  w = 240
  l = 240
  res = 0.025
  i = w/2 + goal_x/res
  j = l/2 - goal_y/res
  z = i*w + j
  
  print("callback")
  
  msg.header.stamp = rospy.Time.now()
  
  array = len(msg.data) * [0]
  
#  for i in range(5):
#    array[i] = 100

  array[math.ceil(z)] = 100
  
  msg.data = array
  
  for i in range(0, 10):
    occ_grid_pub.publish(msg)
    rospy.sleep(0.5)
  
  

rospy.init_node("test_node", anonymous = True)

occ_grid_pub = rospy.Publisher('costmap_test_topic', OccupancyGrid, queue_size=1)

occ_grid_sub = rospy.Subscriber("/rt_costmap_node/costmap/costmap", OccupancyGrid, callback)

rospy.spin()

