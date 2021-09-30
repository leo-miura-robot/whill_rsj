#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import tf2_ros
import math
import numpy as np
import random
import traceback

import hsrb_interface

from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix

from hsrb_interface import geometry
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from geometry_msgs.msg import WrenchStamped, Twist

from states.finish_moving import FinishMovingNoConfirmation
from states.move_robot import MoveRobot

from common import speech
from common.smach_states import *
from common import rospose_to_tmcpose

#from pocketsphinx_jsgf import PocketSphinxClient
#sphinx = PocketSphinxClient()

#TamaGoya Navigation
from takeshi_tools.nav_tool_lib import nav_module

##################################################

# Main
robot = hsrb_interface.Robot()

whole_body = robot.get("whole_body")
#omni_base = robot.get("omni_base")
omni_base = nav_module("hsr") #New initalisation by Pumas
gripper = robot.get('gripper')
tf_buffer = robot._get_tf2_buffer()

default_tts = speech.DefaultTTS()
console = speech.Console()

SAY = default_tts.say

whole_body.move_to_neutral()
rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['success', 'failure'])

  sm.userdata.robot_poses = []

  with sm:

	smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=120.,
				say_fn=SAY,
				prompt_msg="Push the hand to start",
				success_msg=""),
				transitions={'success'	: 'MOVEROBOT',
					     'timeout'	: 'failure',
					     'failure'	: 'failure'})

        smach.StateMachine.add('MOVEROBOT', MoveRobot(tf_buffer = tf_buffer),
                               transitions={'finishing': 'FINISHMOVINGNOCONFIRMATION',
					    'failure': 'failure'})

	#!!!IMPORTANT: learns the pose in userdata.robot_poses
        smach.StateMachine.add('FINISHMOVINGNOCONFIRMATION', FinishMovingNoConfirmation(robot),
                               transitions={'finish': 'success',
					    'failure': 'failure'})


  return sm

sm = create_sm()

outcome = sm.execute()

if outcome == 'success':
	SAY('I finished the task.')
	#rospy.sleep(2)
	rospy.loginfo('I finished the task.')
else:
	SAY('Sorry, something happend. I did not finish the task.')
	rospy.signal_shutdown('Some error occured.')
