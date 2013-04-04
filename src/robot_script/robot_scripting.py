#!/usr/bin/env python

'''
Module for simple scripting of robot actions. Uses Austin Hendrix's
pr2_simple_interface underneath. Adds:
   - Control of individual arm joints, in addition to the all-joints versions
   - Sensor reading, including state of each arm joint
   - Arrangement to anticipate scripting for other types of robots
'''

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

	# ----------------  Abstract Class RobotScript   -----------------------

class RobotScript(object):
	'''
	Abstract class: Scripting for robots. Subclasses implement scripting
	for different types of robots. This class doesn't have much now,
	but might get enriched when different robot is addressed.
	'''
	
	# ----------------  Private Methods and Classes   -----------------------

	def degree2rad(self, deg):
		return numpy.pi*deg/180.0

	def rad2degree(self, rad):
		return 180.0*rad/numpy.pi
	

	# ----------------  Class Pr2RobotScript  -----------------------
	
class PR2RobotScript(RobotScript):
	'''
	Scripting for PR2 robots. 
	'''
	gripper  = pr2_simple_interface.Gripper()
	robotArm = pr2_simple_interface.RobotArm()
	torso    = pr2_simple_interface.Torso()
	head     = pr2_simple_interface.Head()
	
	sensor_observer = None
	initialized = False
	
	LEFT  = pr2_simple_interface.LEFT
	RIGHT = pr2_simple_interface.RIGHT
	BOTH  = pr2_simple_interface.BOTH

	# ----------------  Public Methods -----------------------

	def __init__(self):
		raise RuntimeError("RobotScript cannot be instantiated. Use the static methods.")
	
	@staticmethod	
	def initialize():

		if PR2RobotScript.initialized:
			return

		rospy.init_node('robot_scripts', anonymous=True)
		pr2_simple_interface.start(d=True)
		sensor_observer = PR2RobotScript.PR2SensorObserver()
		sensor_observer.start()
		PR2RobotScript.initialized = True;
		
		#PR2RobotScript.tiltHead(90);
		#PR2RobotScript.panHead(150);
		#PR2RobotScript.openGripper(RIGHT)
		#PR2RobotScript.closeGripper(RIGHT)

	@staticmethod
	def tiltHead(newVal):
		if not PR2RobotScript.initialized:
			PR2RobotScript.initialize()
		pass
		
	@staticmethod		
	def openGripper(side):
		if not PR2RobotScript.initialized:
			PR2RobotScript.initialize()
		PR2RobotScript.gripper.release(side)

	@staticmethod
	def closeGripper(side):
		if not PR2RobotScript.initialized:
			PR2RobotScript.initialize()
		PR2RobotScript.gripper.close(side)

	@staticmethod
	def panHead(newVal):
		if not PR2RobotScript.initialized:
			PR2RobotScript.initialize()
		pass
		
	# ----------------  Class Pr2SensorObserver   -----------------------		
		
	class PR2SensorObserver(object):
		'''
		Implements sensor reading from PR2 robot.
		'''
	
		def __init__(self):
		    super(PR2RobotScript.PR2SensorObserver, self).__init__()
		    self._latest_joint_state = None
		    self._subscriber = None
		    self._jointPositions = {}
		    self._jointVelocities = {}
		
		def start(self):
		    rospy.loginfo('PR2SensorObserver.start()')
		    if self._subscriber is not None:
		        self._subscriber.unregister()
		    self._subscriber = rospy.Subscriber('/joint_states', JointState, self._receive_joint_states)
		
		def stop(self):
		    rospy.loginfo('PR2SensorObserver.stop()')
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
