#! /usr/bin/env python3

import rospy
import actionlib
import time
import math

from remote_teleop_robot_backend.msg import TurnInPlaceAction, TurnInPlaceActionGoal, TurnInPlaceResult
from remote_teleop_robot_backend.msg import PointClickNavAction, PointClickNavActionGoal, PointClickNavResult
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#-----------------------------------------------------------------------------------------------------------------------

class NavController():
    
    def __init__(self):
        pass
        
    
    def at_goal(self):
        pass
        
    def control_to(self):
        pass

#-----------------------------------------------------------------------------------------------------------------------

class RotationController():
    
    def __init__(self):
        pass
        
#    def calc_time_to_angle(self, angle, angular_velocity, is_radians):
#        
#        if is_radians is True:
#            return angle / angular_velocity
#        else:
#            return angle * math.pi / 180 / angular_velocity
#    
#    def at_turn_goal(self, start_time, time_to_angle):
#        
#        if time.time() - start_time => time_to_angle:
#            return True
#        else:
#            return False
#            
#    def turn_robot(self, angle, angular_velocity, is_left, is_radians, _turn_in_place_pub):
#        msg = Twist()
#        msg.linear.x = 0.0
#        msg.linear.y = 0.0
#        msg.linear.z = 0.0
#        msg.angular.x = 0.0
#        msg.angular.y = 0.0
#        
#        if is_left == True:
#            msg.angular.z = angular_velocity
#        else:
#            msg.angular.z = -angular_velocity
#        
#        now = time.time()
#        
#        while self.at_turn_goal(now, self.calc_time_to_angle(angle, angular_velocity, is_radians)) is False:
#            _turn_in_place_pub.publish(msg)
#        
#        msg.angular.z = 0.0
#        _turn_in_place_pub.publish(msg)
#        
#        return
#        
#    def test(self):
        
                
        
#-----------------------------------------------------------------------------------------------------------------------

class RemoteTeleopClass():

    def __init__(self, turn_in_place_pub_topic, set_nav_goal_pub_topic, odom_sub_topic):
        # create messages that are used to publish result
        self._turn_in_place_result = TurnInPlaceResult()
        self._set_nav_goal_result = PointClickNavResult()
        
        # create the turn in place action server and start it
        self._turn_in_place_server = actionlib.SimpleActionServer("turn_in_place_as", TurnInPlaceAction, self.turn_in_place_callback, False)
        self._turn_in_place_server.start()
        
        # create the set nav goal action server and start it
        self._set_nav_goal_server = actionlib.SimpleActionServer("point_click_nav_as", PointClickNavAction, self.set_nav_goal_callback, False)
        self._set_nav_goal_server.start()
        
        # create the turn in place cmd vel publisher
        self._turn_in_place_pub = rospy.Publisher(turn_in_place_pub_topic, Twist, queue_size=1)
        
        # create the set nav goal cmd vel publisher
        self._set_nav_goal_pub = rospy.Publisher(set_nav_goal_pub_topic, Twist, queue_size=1)
        
        # create the odometry subscriber
        self._odom_sub = rospy.Subscriber(odom_sub_topic, Odometry, self.odom_callback)
        
        # other variables
        self.angle = 0.0
        self.turn_left = True
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        

    def turn_in_place_callback(self, msg):
        # TODO implement functionality to gray out the rviz plugin buttons
        # while a turn is being executed so no more than 1 command can be
        # sent at a time
    
        # get the inputs from Rviz and store them in variables        
        self.angle = msg.degrees
        self.turn_left = msg.turn_left
        #print(self.angle)
        # TODO get the linear/angular velocity updates here
        self.linear_velocity = 0.0
        self.angular_velocity = 0.5
        
        # convert from degrees to radians
        self.angle = self.angle * math.pi / 180
                
        self.turn_in_place()
        
        self._turn_in_place_result.success = True
        self._turn_in_place_server.set_succeeded(self._turn_in_place_result)
        
        
        
    
    def set_nav_goal_callback(self, msg):
        pass
        
    
    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        #print(f"Yaw = {self.yaw}")

    def turn_in_place(self):
        command = Twist()
        if self.turn_left == False:
            print("TURN RIGHT")
            goal_yaw = self.yaw - self.angle
            print(goal_yaw)
            
            if goal_yaw < -math.pi:
                goal_yaw += 2*math.pi
                print(goal_yaw)
        else:
            print("TURN LEFT")
            goal_yaw = self.yaw + self.angle
            print(goal_yaw)
            
            if goal_yaw > math.pi:
                goal_yaw -= 2*math.pi
                print(goal_yaw)
                
        print(goal_yaw, self.yaw)
        while abs(goal_yaw - self.yaw) > 0.08: 
            command.angular.z = self.angular_velocity * (goal_yaw - self.yaw)
            #print(goal_yaw, self.yaw, command.angular.z)
            if self.turn_left == True and command.angular.z < 0.0:
                command.angular.z *= -1
            elif self.turn_left == False and command.angular.z > 0.0:
                command.angular.z *= -1
            print(self.turn_left, command.angular.z, goal_yaw, self.yaw)
            self._turn_in_place_pub.publish(command)
        
        print(goal_yaw, self.yaw)
        command.angular.z = 0.0
        self._turn_in_place_pub.publish(command)
        print("got here")
#        print(command.angular.z)
        

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('remote_teleop_node', anonymous=True)
    
    # initialize the cmd_vel publishers
    turn_in_place_pub_topic = '/cmd_vel'
    set_nav_goal_pub_topic = '/cmd_vel'
    
    # initialize the subscribers
    odom_sub_topic = '/odom'

    # initialize the class
    remote_teleop = RemoteTeleopClass(turn_in_place_pub_topic, set_nav_goal_pub_topic, odom_sub_topic)

    # keep the program from exiting before the node is stopped
    rospy.spin()
