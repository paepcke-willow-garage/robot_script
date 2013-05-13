#!/usr/bin/env python

'''
Module for simple scripting of robot actions. Uses Austin Hendrix's
pr2_simple_interface underneath. Adds:
   - Control of individual arm joints, in addition to the all-joints versions
   - Sensor reading, including state of each arm joint
   - Arrangement to anticipate scripting for other types of robots
   - Base motion
   - Event injection into scripts
   
@author: Andreas Paepcke, based on Austin Hendrix' pr2_simple_interface, and Maya Cakmak's fake_robot.py
'''

import roslib
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
import copy
from threading import Lock

# ROS libraries
import rospy
import tf
from tf import TransformListener, TransformBroadcaster
#from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import JointState 
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

import pr2_simple_interface

# Rate at which base motion gets refreshed:
UPDATE_RATE = 0.1; # seconds
#UPDATE_RATE = 0.05; # seconds

class Units:
    ANGULAR  = 0;
    DISTANCE = 1;

class Tolerances:
    ANGULAR  = 10;     # +/- degrees
    DISTANCE = 0.02   # +/- meters
    BASE     = 0.2    # +/- meters (Base less accurate via odometry)

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
    
    # Used for waiting:
    done_condition = threading.Condition();
    
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
    base     = None;
    
    sensor_observer = None
    initialized = False
    
    transformListener = None
    
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

    # ----------------  Public Methods -----------------------

    def __init__(self):
        raise RuntimeError("RobotScript cannot be instantiated. Use the static methods.")
    
    @staticmethod    
    def initialize():

        if PR2RobotScript.initialized:
            return

        rospy.init_node('robot_scripts', anonymous=True)
        pr2_simple_interface.start(d=True)
        PR2RobotScript.base = PR2Base()
        PR2RobotScript.sensor_observer = PR2RobotScript.PR2SensorObserver()
        PR2RobotScript.sensor_observer.start()
        PR2RobotScript.transformListener = tf.TransformListener()
        try:
            pass
            #*****rospy.wait_for_service("spawn", timeout=2.0)
        except:
            raise RuntimeError("Transform listener does not appear to be running.");
        
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
        if sensorName == "base":
            reading.w =  RobotScript.rad2degree(reading.w)
            return reading
        if PR2RobotScript.pr2_joint_units[sensorName] == Units.ANGULAR:
           return RobotScript.rad2degree(reading)
        else:
           return reading

    @staticmethod
    def waitFor(jointNames, settings, safetyTimeout=rospy.Duration(30.0)):

        if not isinstance(jointNames, types.ListType):
            jointNames = [jointNames]
        if not isinstance(settings, types.ListType):
            settings = [settings]
        
        # Sanity check:
        for jointName in jointNames:
            if jointName not in PR2RobotScript.pr2_joints:
                raise ValueError("PR2 has no joint named '%s'." % jointName);
        
        timeout_time = rospy.get_rostime() + safetyTimeout
        loop_period = rospy.Duration(0.1)
        
        with RobotScript.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if safetyTimeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    rospy.logwarn("Safety timeout: one of joints %s did not reach its intended position." % str(jointNames));
                    return;
                done = True
                for i,jointName in enumerate(jointNames):
                    if not aboutEq(jointName, settings[i]):
                        done = False
                        break
                if done:
                    return;
    
                if time_left > loop_period or safetyTimeout == rospy.Duration():
                    time_left = loop_period
    
                RobotScript.done_condition.wait(time_left.to_sec())

    @staticmethod
    def tiltHead(newTilt, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the pan that we are about
        # to do will be transient:
        PR2RobotScript.head.wait_for()
        currRot = PR2RobotScript.getSensorReading('head_pan_joint');
        PR2RobotScript.head.look(currRot, newTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_tilt_joint', newTilt);
        
    @staticmethod
    def panHead(newPan, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the tilt that we are about
        # to do will be transient:
        PR2RobotScript.head.wait_for()
        currTilt = PR2RobotScript.getSensorReading('head_tilt_joint');
        PR2RobotScript.head.look(newPan, currTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_pan_joint', newPan);
     
    @staticmethod
    def lookAt(pan, tilt, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.head.look(pan, tilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_pan_joint', pan);
            PR2RobotScript.waitFor('head_tilt_joint', tilt);
     
        
    @staticmethod
    def rotateHead(newVal, duration=1.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        # If currently moving head, must wait for that done,
        # else the reading for the tilt that we are about
        # to do will be transient:
        PR2RobotScript.head.wait_for()
        currTilt = PR2RobotScript.getSensorReading('head_tilt_joint');
        PR2RobotScript.head.look(newVal, currTilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor(['head_tilt_joint', 'head_pan_joint'], [currTilt, newVal]);
        
    @staticmethod        
    def openGripper(side):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.release(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint', 0.1);
        else:
            PR2RobotScript.waitFor('r_gripper_joint', 0.1);

    @staticmethod
    def closeGripper(side):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.close(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint', 0);
        else:
            PR2RobotScript.waitFor('r_gripper_joint', 0);

    @staticmethod
    def setTorso(height, duration=10.0, wait=True):
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.torso.set(height, dur=duration)
        if wait:
            PR2RobotScript.waitFor('torso_lift_joint', height);

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
            PR2RobotScript.waitFor(jointNames, newAngles)
          
    @staticmethod
    def moveBase(place=(0.0,0.0,0.0), rotation=0.0, duration=3.0, wait=True):
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
  
        motionThread = RobotBaseMotionThread();
        motionThread.start(Quaternion(place[0], place[1], place[2], rotation), duration);
        if wait:
            PR2RobotScript.base.wait_for();
        return;


    # ----------------  Class Pr2SensorObserver   -----------------------        
        
    class PR2SensorObserver(object):
        '''
        Implements sensor reading from PR2 robot.
        '''
    
        def __init__(self):
            super(PR2RobotScript.PR2SensorObserver, self).__init__()
            self._latest_joint_state = None
            self._subscriber = None
            self._odom_subscriber = None
            self._jointPositions = {}
            self._jointVelocities = {}
            self._base_xyzw = None
            self.basePositionVarLock   = Lock();
            self.jointPositionsVarLock = Lock();

        def getSensorReading(self, sensorName):
            if len(self._jointPositions) == 0:
                # _receive_joint_states wasn't called yet. Give it a chance
                rospy.sleep(0.5);
                if len(self._jointPositions) == 0:
                    raise RuntimeError("PR2SensorObserver not initialized, or no joint states being processed.");
            if self._base_xyzw is None:
                # _receive_odometry_states wasn't called yet. Give it a chance
                rospy.sleep(0.5);
                if self._base_xyzw is None:
                    raise RuntimeError("PR2SensorObserver not initialized, or no robot base position states being processed.");
            try:
                if sensorName == "base":
                    with self.basePositionVarLock:
                        basePosCopy = copy.copy(self._base_xyzw);
                        return basePosCopy;
                else:
                    with self.jointPositionsVarLock:
                        jointPosCopy = copy.copy(self._jointPositions[sensorName]);
                        return jointPosCopy;
            except KeyError:
                raise ValueError("Sensing for '%s' is not implemented. Maybe the sensor name is wrong?\nLegal names are:\n%s" % (sensorName,PR2RobotScript.jointsStr));
        
        def start(self):
            rospy.loginfo('PR2SensorObserver.start()')
            if self._subscriber is not None:
                self._subscriber.unregister()
            if self._odom_subscriber is not None:
                self._odom_subscriber.unregister()
            self._subscriber      = rospy.Subscriber('/joint_states', JointState, self._receive_joint_states)
            self._odom_subscriber = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._receive_odometry_states)
        
        def stop(self):
            rospy.loginfo('PR2SensorObserver.stop()')
            if self._subscriber is not None:
                self._subscriber.unregister()
                self._subscriber = None
            if self._odom_subscriber is not None:
                self._odom_subscriber.unregister()
                self._odom_subscriber = None
        
        def _receive_joint_states(self, joint_state_msg):
            self._latest_joint_state = joint_state_msg
            with self.jointPositionsVarLock:
                for i in range(len(joint_state_msg.name)):
                    self._jointPositions[joint_state_msg.name[i]] = joint_state_msg.position[i]
                    self._jointVelocities[joint_state_msg.name[i]] = joint_state_msg.velocity[i]
                    
        def _receive_odometry_states(self, the_odom_msg):
            #print(str(odom_msg))
            odom_msg = the_odom_msg;
            with self.basePositionVarLock:
                self._base_xyzw = Quaternion(odom_msg.pose.pose.position.x,
                                             odom_msg.pose.pose.position.y,
                                             odom_msg.pose.pose.position.z,
                                             odom_msg.pose.pose.orientation.w);
            
class RobotBaseMotionThread(threading.Thread):
    
    oneThreadRunning = None;
    
    def start(self, targetQuaternion, motionDuration):
        if RobotBaseMotionThread.oneThreadRunning is not None:
            #raise RuntimeError("Must stop robot motion thread before starting a new one: Call self.robotMotionThread.stop()");
            RobotBaseMotionThread.oneThreadRunning.stop();
                    
        self.targetQuaternion = targetQuaternion;
        self.motionDuration = motionDuration
        self.keepRunning = True;
        super(RobotBaseMotionThread, self).start();
        
        
    def run(self):
        RobotBaseMotionThread.oneThreadRunning = self;
        targetX = self.targetQuaternion.x;
        targetY = self.targetQuaternion.y;        
        targetRotRad = PR2RobotScript.degree2rad(self.targetQuaternion.w)
        rotToleranceRad = PR2RobotScript.degree2rad(Tolerances.ANGULAR)
        rotTraveled  = 0.0
        xTraveled = 0.0
        yTraveled = 0.0
        rotGoalReached = False
        xGoalReached   = False
        yGoalReached   = False
        twistMsg  = Twist();
        twistMsg.linear  = Vector3(self.targetQuaternion.x, self.targetQuaternion.y, self.targetQuaternion.z);  
        twistMsg.angular = Vector3(0.0,0.0, PR2RobotScript.degree2rad(self.targetQuaternion.w));
        # Get initial positions: translation as Vector3 (x,y,z), and rot as quaternion:
        while self.keepRunning and not rospy.is_shutdown():
            try:
                (initialTrans, initialRot) = PR2RobotScript.transformListener.lookupTransform('base_footprint', 'odom_combined', rospy.Time(0));
                # 
                prevRot   = initialRot[3]    # scalar rotation radians
                prevX     = initialTrans[0]   # scalar meters
                prevY     = initialTrans[1]   # scalar meters
                # Need to detect later if amount traveled slips past the target,
                # because of bad sampling luck:
                
                if targetRotRad > initialRot[3]:
                    rotTargetGreater = True
                else:
                     rotTargetGreater = False

                if targetX > initialTrans[0]:
                    xTargetGreater = True
                else:
                    xTargetGreater = False
                
                if targetY > initialTrans[1]:
                    yTargetGreater = True
                else:
                    yTargetGreater = False
                
                break;
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
        # Keep whipping the base controller till goals are reached:
        while self.keepRunning and not rospy.is_shutdown():
            try:
                # Where is the robot after this iteration's sleep:
                (trans,rot) = PR2RobotScript.transformListener.lookupTransform('base_footprint', 'odom_combined', rospy.Time(0));
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            # Compute total rotation traveled:
            if targetRotRad > 0:
                rotTraveled += abs(rot[3] - prevRot)
            else:
                rotTraveled -= abs(rot[3] - prevRot)

            if targetX > 0:
                xTraveled += abs(trans[0] - prevX)
            else:
                xTraveled -= abs(trans[0] - prevX)

            if targetY > 0:
                yTraveled += abs(trans[1] - prevY)
            else:
                yTraveled -= abs(trans[1] - prevY)

                
            # Is rotation goal reached?
            if (abs(targetRotRad - rotTraveled) <= rotToleranceRad) or\
                (rotTargetGreater and targetRotRad <= rotTraveled) or\
                (not rotTargetGreater and targetRotRad > rotTraveled):
                rotGoalReached = True;
                twistMsg.angular = Vector3(0.0,0.0,0.0);
            
            # Are x and y goals reached?
            if (abs(targetX - xTraveled) <= Tolerances.DISTANCE) or\
                (xTargetGreater and targetX <= xTraveled) or\
                (not xTargetGreater and targetX > xTraveled):
                xGoalReached = True;
                twistMsg.linear = Vector3(0.0, twistMsg.linear.y, 0.0);
            
            if (abs(targetY - yTraveled) <= Tolerances.DISTANCE) or\
                (yTargetGreater and targetY <= yTraveled) or\
                (not yTargetGreater and targetY > yTraveled):
                yGoalReached = True;
                twistMsg.linear = Vector3(twistMsg.linear.x, 0.0, 0.0);
            
            if rotGoalReached and xGoalReached and rotGoalReached:
                self.stop();
            
            prevRot = rot[3]
            prevX   = trans[0]
            prevY   = trans[1]
            
            PR2RobotScript.baseMovementPublisher.publish(twistMsg);
            time.sleep(UPDATE_RATE)
            continue;           
        
    def stop(self):
        self.keepRunning = False;
        RobotBaseMotionThread.oneThreadRunning = None;


class PR2Base(object):
    
    def wait_for(self):
        while RobotBaseMotionThread.oneThreadRunning is not None:
            time.sleep(0.2);

def aboutEq(sensorName, val):
    sensorVal = PR2RobotScript.getSensorReading(sensorName);
    
    if sensorName == "base":
        # val is a Quaternion test all four values:
        if not doComparison(sensorVal.x, val.x, Tolerances.BASE) or\
           not doComparison(sensorVal.y, val.y, Tolerances.BASE) or\
           not doComparison(sensorVal.z, val.z, Tolerances.BASE) or\
           not doComparison(sensorVal.w, val.w, Tolerances.BASE):
            return False
        else:
            return True

    # A joint angle: 
    
    if PR2RobotScript.pr2_joint_units[sensorName] == Units.ANGULAR:
        tolerance = Tolerances.ANGULAR; # degrees
    else:
        tolerance = Tolerances.DISTANCE # meters
    
    
    return doComparison(sensorVal, val, tolerance)
        
def doComparison(sensorVal, targetVal, tolerance):        
    if (abs(sensorVal - targetVal) <= tolerance):
        return True;
    else:
        return False;

if __name__ == "__main__":
    
    rospy.init_node('robot_scripts', anonymous=True)
    #robot = RobotScript()
    rospy.spin()
