#! /usr/bin/env python3

import rospy
import actionlib
from action_remote_teleop.msg import TurnInPlaceAction, TurnInPlaceGoal, TurnInPlaceFeedback, TurnInPlaceResult

class TurnInPlaceClass():
	
	# create messages that are used to publish feedback/results
	_feedback = TurnInPlaceFeedback()
	_result = TurnInPlaceResult()
	
	def __init__(self):
		# creates the action server
		self._actionServer = 
