#! /usr/bin/env python3

import rospy
import actionlib

from remote_teleop.msg import TurnInPlaceAction, TurnInPlaceGoal, TurnInPlaceResult

class TurnInPlaceClass():

    # create messages that are used to publish result
    _result = TurnInPlaceResult()

    def __init__(self):
        # create the action server and start it
        self._turn_in_place_server = actionlib.SimpleActionServer("turn_in_place_as", TurnInPlaceAction, self.callback, False)
        self._turn_in_place_server.start()

    def callback(self, msg):
        #TODO: put something here
        print(msg)

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('remote_teleop_node', anonymous=True)

    # initialize the class
    turn_in_place = TurnInPlaceClass()

    # keep the program from exiting before the node is stopped
    rospy.spin()
