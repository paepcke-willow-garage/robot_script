#!/usr/bin/env python
import roslib
roslib.load_manifest('proximity_animation')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

# Generic libraries
import sys,os,time
import numpy, math

# ROS libraries
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import JointState 

import pr2_simple_interface
from pr2_simple_interface import Gripper
from pr2_simple_interface import RobotArm
from pr2_simple_interface import Torso
from pr2_simple_interface import Head

from pr2_simple_interface import LEFT,RIGHT,BOTH

class RobotScript(object):
	'''
	Scripting for PR2 robots.
	'''
	gripper  = Gripper()
	robotArm = RobotArm()
	torso    = Torso()
	head     = Head()
	
	joint_observer = None
	initialized = False
	
	LEFT  = pr2_simple_interface.LEFT
	RIGHT = pr2_simple_interface.RIGHT
	BOTH  = pr2_simple_interface.BOTH

	# ----------------  Public Methods -----------------------

	def __init__(self):
		raise RuntimeError("RobotScript cannot be instantiated. Use the static methods.")
	
	@staticmethod	
	def initialize():

		if RobotScript.initialized:
			return
		rospy.init_node('robot_scripts', anonymous=True)
		pr2_simple_interface.start(d=True)
		RobotScript.joint_observer = JointObserver()
		RobotScript.joint_observer.start()
		RobotScript.initialized = True;
		
		#RobotScript.tiltHead(90);
		#RobotScript.panHead(150);
		#RobotScript.openGripper(RIGHT)
		#RobotScript.closeGripper(RIGHT)

	@staticmethod
	def tiltHead(newVal):
		if not RobotScript.initialized:
			RobotScript.initialize()
		pass
		
	@staticmethod		
	def openGripper(side):
		if not RobotScript.initialized:
			RobotScript.initialize()
		RobotScript.gripper.release(side)

	@staticmethod
	def closeGripper(side):
		if not RobotScript.initialized:
			RobotScript.initialize()
		RobotScript.gripper.close(side)

	@staticmethod
	def panHead(newVal):
		if not RobotScript.initialized:
			RobotScript.initialize()
		pass
		
	# ----------------  Private Methods and Classes   -----------------------

	def degree2rad(self, deg):
		return numpy.pi*deg/180.0

	def rad2degree(self, rad):
		return 180.0*rad/numpy.pi

class JointObserver(object):

    def __init__(self):
        super(JointObserver, self).__init__()
        self._latest_joint_state = None
        self._subscriber = None
        self._jointPositions = {}
        self._jointVelocities = {}

    def start(self):
        rospy.loginfo('JointObserver.start()')
        if self._subscriber is not None:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber('/joint_states', JointState, self._receive_joint_states)

    def stop(self):
        rospy.loginfo('JointObserver.stop()')
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    def _receive_joint_states(self, joint_state_msg):
		self._latest_joint_state = joint_state_msg
		for i in range(len(joint_state_msg.name)):
			self._jointPositions[joint_state_msg.name[i]] = joint_state_msg.position[i]
			self._jointVelocities[joint_state_msg.name[i]] = joint_state_msg.velocity[i]
		
		#rospy.loginfo("Joint state: " + str(self._latest_joint_state))


if __name__ == "__main__":
    rospy.init_node('robot_scripts', anonymous=True)
    #robot = RobotScript()
    rospy.spin()
