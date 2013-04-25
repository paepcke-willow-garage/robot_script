#!/usr/bin/env python

'''
Module for simple scripting of robot actions. Uses Austin Hendrix's
pr2_simple_interface underneath. Adds:
   - Control of individual arm joints, in addition to the all-joints versions
   - Sensor reading, including state of each arm joint
   - Arrangement to anticipate scripting for other types of robots
   
@author: Andreas Paepcke, based on Austin Hendrix' pr2_simple_interface, and Maya Cakmak's fake_robot.py
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
from functools import partial
import types

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

class Units:
    ANGULAR  = 0;
    DISTANCE = 1;

class Tolerances:
    ANGULAR  = 6;     # +/- degrees
    DISTANCE = 0.02   # +/- meters

class FullPose(object):
    '''
    Container for a Pose instance and a Quarternion.
    They together define where the base is to go,
    and how to turn:
    '''
    def __init__(self, place=(0.0,0.0,0.0), rotation=0.0):
        self.linear  = Point(place[0],place[1],place[2])
        self.angular = Quaternion(0.0,0.0,RobotScript.degree2rad(rotation),0.0)
        self.pose    = Pose(self.linear, self.angular)

    # ----------------  ABSTRACT CLASS ROBOTSCRIPT   -----------------------

class RobotScript(object):
    '''
    Abstract class: Scripting for robots. Subclasses implement scripting
    for different types of robots. This class doesn't have much now,
    but might get enriched when different robot is addressed.
    '''
    
    # ----------------  Private Methods and Classes   -----------------------

    @staticmethod
    def degree2rad(deg):
        return numpy.pi*deg/180.0

    @staticmethod
    def rad2degree(rad):
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
    
    pr2_joints = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 
                  'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 
                  'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 
                  'laser_tilt_mount_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 
                  'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 
                  'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 
                  'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 
                  'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 
                  'l_gripper_motor_slider_joint']    

    lArmJoints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
    rArmJoints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]

    pr2_joint_units = {}

    pr2_joint_wait_methods = {}

    # ----------------  Public Methods -----------------------

    def __init__(self):
        raise RuntimeError("RobotScript cannot be instantiated. Use the static methods.")
    
    @staticmethod    
    def initialize():

        if PR2RobotScript.initialized:
            return

        rospy.init_node('robot_scripts', anonymous=True)
        pr2_simple_interface.start(d=True)
        PR2RobotScript.sensor_observer = PR2RobotScript.PR2SensorObserver()
        PR2RobotScript.sensor_observer.start()
        PR2RobotScript.initialized = True;
        
        # Build a string with all legal joints/sensors, four to a line
        # for error msgs:
        PR2RobotScript.jointsStr = "";
        jointsPerLine = 4;
        numJoints = len(PR2RobotScript.pr2_joints);
        numFullLines = int(numJoints / jointsPerLine);
        for i in range(0, numJoints-jointsPerLine, jointsPerLine):
            PR2RobotScript.jointsStr += PR2RobotScript.pr2_joints[i] + ', ';
            PR2RobotScript.jointsStr += PR2RobotScript.pr2_joints[i+1] + ', ';
            PR2RobotScript.jointsStr += PR2RobotScript.pr2_joints[i+2] + ', ';
            PR2RobotScript.jointsStr += PR2RobotScript.pr2_joints[i+3] + ',\n';
        for i in  range(numFullLines * jointsPerLine, numJoints):
            PR2RobotScript.jointsStr += PR2RobotScript.pr2_joints[i] + ', ';
        
        # Ditch trailing comma:
        PR2RobotScript.jointsStr = PR2RobotScript.jointsStr[:-2];   

        # Build dict of methods that wait for particular joints to finish:
        for jName in ['head_tilt_joint', 'head_pan_joint']:
            PR2RobotScript.pr2_joint_wait_methods[jName] = PR2RobotScript.head.wait_for;
        PR2RobotScript.pr2_joint_wait_methods['torso_lift_joint'] = PR2RobotScript.torso.wait_for;

        for jName in ['l_shoulder_pan_joint', 'l_shoulder_lift_joint',
                      'l_upper_arm_roll_joint', 'l_elbow_flex_joint',
                      'l_forearm_roll_joint', 'l_wrist_flex_joint',
                      'l_wrist_roll_joint']:
            PR2RobotScript.pr2_joint_wait_methods[jName] = partial(PR2RobotScript.robotArm.wait_for, PR2RobotScript.LEFT);
        
        for jName in ['r_shoulder_pan_joint', 'r_shoulder_lift_joint',
                      'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
                      'r_forearm_roll_joint', 'r_wrist_flex_joint',
                      'r_wrist_roll_joint']:
            PR2RobotScript.pr2_joint_wait_methods[jName] = partial(PR2RobotScript.robotArm.wait_for, PR2RobotScript.RIGHT);
            
        PR2RobotScript.pr2_joint_wait_methods['l_gripper_joint'] = partial(PR2RobotScript.robotArm.wait_for, PR2RobotScript.LEFT);
        PR2RobotScript.pr2_joint_wait_methods['r_gripper_joint'] = partial(PR2RobotScript.robotArm.wait_for, PR2RobotScript.RIGHT);

        # Build dict of jointName-->Unit. Most joints are angular, so 
        # we fill the whole dict with Units.ANGULAR, replacing the
        # few metric joints' entries:
        for jName in PR2RobotScript.pr2_joints:
            PR2RobotScript.pr2_joint_units[jName] = Units.ANGULAR;
        for jName in ['l_gripper_joint', 'r_gripper_joint', 'torso_lift_joint']:
            PR2RobotScript.pr2_joint_units[jName] = Units.DISTANCE;
            
        # Publisher for base movements        
        PR2RobotScript.baseMovementPublisher = rospy.Publisher('base_controller/command', Twist)

    @staticmethod
    def getSensorReading(sensorName):
        '''
        Returns reading of the sensor with the given name. Example: 'torso_lift_link'.
        @param sensorName: Name of sensor to read
        @type sensorName: string
        @return: Sensor reading
        @rtype: int
        @raise ValueError: no sensor with given name exists, at least in the implementation. 
        '''
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        reading = PR2RobotScript.sensor_observer.getSensorReading(sensorName); 
        if PR2RobotScript.pr2_joint_units[sensorName] == Units.ANGULAR:
           return RobotScript.rad2degree(reading)
        else:
           return reading

    @staticmethod
    def waitFor(jointName):
        try:
            PR2RobotScript.pr2_joint_wait_methods[jointName]();
        except KeyError:
            raise ValueError("No joint named '%s' has a 'wait' facility." % jointName);

    @staticmethod
    def tiltHead(newTilt, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the pan that we are about
        # to do will be transient:
        PR2RobotScript.waitFor('head_pan_joint')
        currRot = PR2RobotScript.getSensorReading('head_pan_joint');
        PR2RobotScript.head.look(currRot, newTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_tilt_joint');
        
    @staticmethod
    def panHead(newPan, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the tilt that we are about
        # to do will be transient:
        PR2RobotScript.waitFor('head_tilt_joint')
        currTilt = PR2RobotScript.getSensorReading('head_tilt_joint');
        PR2RobotScript.head.look(newPan, currTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_pan_joint');
        
    @staticmethod
    def rotateHead(newVal, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the tilt that we are about
        # to do will be transient:
        PR2RobotScript.waitFor('head_tilt_joint')
        currTilt = PR2RobotScript.getSensorReading('head_tilt_joint');
        PR2RobotScript.head.look(newVal, currTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_tilt_joint');
            PR2RobotScript.waitFor('head_pan_joint');
        
    @staticmethod        
    def openGripper(side):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.release(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint');
        else:
            PR2RobotScript.waitFor('r_gripper_joint');

    @staticmethod
    def closeGripper(side):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.close(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint');
        else:
            PR2RobotScript.waitFor('r_gripper_joint');

    @staticmethod
    def setTorso(height, duration=10.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.torso.set(height, dur=duration)
        if wait:
            PR2RobotScript.waitFor('torso_lift_joint');

    @staticmethod
    def moveArmJoint(jointNames, newAngles, duration=2.0, wait=True):
        '''
        Move one or more arm joints to a new angle. 
        @param jointNames: Either a single joint name, or an array of joint names.
        @type jointNames: {jointName | [jointName]}
        @param newAngles: Either a single angle, or an array of angles. There must be as many joint angles as joint names.
        @type newAngles: {angleValue | [angleValue]}
        @param duration: How long to take execution the motion
        @type duration: float
        @param wait: whether to wait till motion is done before returning from method 
        @type wait: boolean
        '''
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
            
        if not isinstance(jointNames, types.ListType):
            jointNames = [jointNames]
        
        if not isinstance(newAngles, types.ListType):
            newAngles = [newAngles]
            
        # Is this a left or right joint?
        try:
            PR2RobotScript.lArmJoints.index(jointNames[0]); 
            side = PR2RobotScript.LEFT
        except ValueError:
            try:
                PR2RobotScript.rArmJoints.index(jointNames[0]);
                side = PR2RobotScript.RIGHT
            except ValueError:
                raise ValueError("PR2 arms have no joint named '%s'" % jointNames[0])
                
        # Init an arm joint array with the new value the one
        # given joint is to take. Then fill in the other joints
        # with their current values:
        newPos = [0]*len(PR2RobotScript.lArmJoints)
        if side == PR2RobotScript.LEFT:
            for pos, jName in enumerate(PR2RobotScript.lArmJoints):
                if jName in jointNames:
                    newPos[pos] = newAngles[jointNames.index(jName)]
                else:
                    newPos[pos] = PR2RobotScript.getSensorReading(jName);
        else:
            for pos, jName in enumerate(PR2RobotScript.rArmJoints):
                if jName in jointNames:
                    newPos[pos] = newAngles[jointNames.index(jName)]
                else:
                    newPos[pos] = PR2RobotScript.getSensorReading(jName);
        PR2RobotScript.robotArm.move_to(newPos, side, dur=duration)
        if wait:
            PR2RobotScript.robotArm.wait_for(side)
          
    @staticmethod
    def moveBase(place=(0.0,0.0,0.0), rotation=0.0, duration=3.0):
        '''
        Move the robot base to a different position, turning its torso at the same time.
        @param place: a tuple with x,y,z coordinates of the destination
        @type place: (float,float,float) 
        @param rotation: simultaneous rotation of the base
        @type rotation: float
        @param duration: duration of motion
        @type duration: float
        '''
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        
        fullPose = FullPose(place, rotation)
        movement = Twist()
        movement.linear = fullPose.linear
        movement.angular = fullPose.angular
        start_time = rospy.get_rostime()
        while rospy.get_rostime() < start_time + rospy.Duration(duration):
            PR2RobotScript.baseMovementPublisher.publish(movement)
        time.sleep(0.01)
        PR2RobotScript.baseMovementPublisher.publish(Twist())  # Stop


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
        
        def getSensorReading(self, sensorName):
            if len(self._jointPositions) == 0:
                # _receive_joint_states wasn't called yet. Give it a chance
                rospy.sleep(0.5);
                if len(self._jointPositions) == 0:
                    raise RuntimeError("PR2SensorObserver not initialized, or no joint states being processed.");
            try:
                return self._jointPositions[sensorName];
            except KeyError:
                raise ValueError("Sensing for '%s' is not implemented. Maybe the sensor name is wrong?\nLegal names are:\n%s" % (sensorName,PR2RobotScript.jointsStr));
        
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

def aboutEq(sensorName, val):
    sensorVal = PR2RobotScript.getSensorReading(sensorName);
    if PR2RobotScript.pr2_joint_units[sensorName] == Units.ANGULAR:
        tolerance = Tolerances.ANGULAR; # degrees
    else:
        tolerance = Tolerances.DISTANCE # meters
    if (abs(sensorVal - val) <= tolerance):
        return True;
    else:
        return False;

if __name__ == "__main__":
    
    rospy.init_node('robot_scripts', anonymous=True)
    #robot = RobotScript()
    rospy.spin()
