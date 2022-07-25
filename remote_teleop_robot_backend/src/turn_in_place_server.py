#! /usr/bin/env python3

import rospy
import actionlib

from remote_teleop.msg import TurnInPlaceAction, TurnInPlaceActionGoal, TurnInPlaceResult

class TurnInPlaceClass():

    def __init__(self):
        # create messages that are used to publish result
        self._result = TurnInPlaceResult()
        # create the action server and start it
        self._turn_in_place_server = actionlib.SimpleActionServer("turn_in_place_as", TurnInPlaceAction, self.callback, False)
        self._turn_in_place_server.start()
        
        # other variables
        self.degrees = 0
        self.turn_left = True
        

    def callback(self, msg):
        # get the inputs from Rviz and store them in variables        
        self.degrees = msg.degrees
        self.turn_left = msg.turn_left
        
        # print out what the current command is
        if self.turn_left == True:
            print(f"L{self.degrees}")
        else:
            print(f"R{self.degrees}")
        
        self._result.success = True
        self._turn_in_place_server.set_succeeded(self._result)
        

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('remote_teleop_node', anonymous=True)

    # initialize the class
    turn_in_place = TurnInPlaceClass()

    # keep the program from exiting before the node is stopped
    rospy.spin()
