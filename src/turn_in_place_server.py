#! /usr/bin/env python3

import rospy
import actionlib

from remote_teleop.msg import TurnInPlaceAction, TurnInPlaceActionGoal, TurnInPlaceResult

class TurnInPlaceClass():

    # create messages that are used to publish result
    _result = TurnInPlaceResult()

    def __init__(self, turn_pub_topic):
        # create the action server and start it
        self._turn_in_place_server = actionlib.SimpleActionServer("turn_in_place_as", TurnInPlaceAction, self.callback, False)
        self._turn_in_place_server.start()
        
        # create publisher for /turn_in_place_as/goal
        self.turn_pub = rospy.Publisher(turn_pub_topic, TurnInPlaceActionGoal, queue_size=10)
        
        # other variables
        self.degrees = 0
        self.turnLeft = True

    def callback(self, msg):
        # TODO: get info from rviz inputs and set them in the variables
        # will need to figure out a way to check if a new event has come in
        
        self.degrees = 5 # update this
        self.turnLeft = True # update this
        
        # create the object
        turn = TurnInPlaceActionGoal()
        
        # set the values for the goal fields
        # TODO: set the values to 0 if there has been no change since the last update OR make sure that this is only being called if there is an update
        turn.goal.degrees = self.degrees
        turn.goal.turnLeft = self.turnLeft
        
        # publish the turn command to the robot to make it turn accordingly
        self.turn_pub.publish(turn)
        
        # print out what the current command is
        if turn.goal.turnLeft is True:
            print("L"+turn.goal.degrees)
        else:
            print("R"+turn.goal.degrees)

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('remote_teleop_node', anonymous=True)
    
    # initialize publisher topics
    turn_pub_topic = '/turn_in_place_as/goal'

    # initialize the class
    turn_in_place = TurnInPlaceClass(turn_pub_topic)

    # keep the program from exiting before the node is stopped
    rospy.spin()
