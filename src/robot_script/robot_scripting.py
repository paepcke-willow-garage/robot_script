#!/usr/bin/env python

'''
Module for simple scripting of robot actions. Uses Austin Hendrix's
pr2_simple_interface underneath. Adds:
   - Control of individual arm joints, in addition to the all-joints versions
   - Sensor reading, including state of each arm joint
   - Arrangement to anticipate scripting for other types of robots
   - Base motion
   - Event injection into scripts
   
see 
   
@author: Andreas Paepcke, based on Austin Hendrix' pr2_simple_interface, and Maya Cakmak's fake_robot.py
'''

import roslib
from mercurial.hgweb.webcommands import static
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
from tf import TransformerROS
#from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from sensor_msgs.msg import JointState 
#from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion

import pr2_simple_interface

# Period at which base motion gets refreshed:
UPDATE_PERIOD = 0.1; # seconds

class Units:
    ANGULAR  = 0;
    DISTANCE = 1;

class AngleUnits:
    RADIANS = 0;
    DEGREES = 1;

class Tolerances:
    ANGULAR       = 10;    # +/- degrees
    DISTANCE      = 0.02   # +/- meters
    BASE_DISTANCE = 0.2    # +/- meters (Base less accurate via odometry)
    BASE_ANGLE    = 15     # +/- degrees

class RotDirection:
    CLOCKWISE = 0;
    COUNTER_CLOCKWISE = 1;

class Sign:
    POSITIVE = 1;
    NEGATIVE = -1

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
    
    # Flag that causes all robot motion commands to do nothing when
    # called. This is useful for faking interrupts for long running 
    # robot_script functions: They run through all commends, but 
    # do nothing, and finish quickly. Real hack!!!
    gearsEngaged = True;
    
    frameTransformer = None
        
    # ----------------  Private Methods and Classes   -----------------------

    #===========================================================================
    
    # For use with TransformROS.transformPose(), which couldn't find frame /odom 
    # or frame /odom_combined:
    # @staticmethod
    # def initialize():
    #    if RobotScript.frameTransformer is None:
    #        RobotScript.frameTransformer = TransformerROS();
    #        # Allow frame transformer to collect state:
    #        rospy.timer.sleep(2.0)
    #===========================================================================

    @staticmethod
    def degree2rad(deg):
        return numpy.pi*deg/180.0

    @staticmethod
    def rad2degree(rad):
        return 180.0*rad/numpy.pi
    
    # For use with TransformROS.transformPose(), which couldn't find frame /odom 
    # or frame /odom_combined:
#    @staticmethod
#    def odometryFromBaseFrame(x,y,z, rot, angleUnits=AngleUnits.DEGREES):
#        '''
#        'base_footprint', 'odom_combined'
#        '''
#        if RobotScript.frameTransformer is None:
#            RobotScript.initialize()
#            
#        msgHeader      = Header(frame_id='base_footprint')
#        posePoint      = Point(x, y, z)
#        if angleUnits == AngleUnits.RADIANS:
#            poseQuaternion = Quaternion(0.0, 0.0, 1.0, rot)
#        else:
#            poseQuaternion = Quaternion(0.0, 0.0, 1.0, RobotScript.degree2rad(rot))
#        pose           = Pose(position=posePoint, orientation=poseQuaternion)
#        poseStamped    = PoseStamped(header=msgHeader, pose=pose)
#        
#        xformedPoseMsg = RobotScript.frameTransformer.transformPose('odom_combined', poseStamped)
#
#        if angleUnits == AngleUnits.RADIANS:
#            resRot = xformedPoseMsg.orientation.w
#        else:
#            resRot = RobotScript.rad2degree(xformedPoseMsg.orientation.w)
#            
#        res = (xformedPoseMsg.pose.position.x,
#               xformedPoseMsg.pose.position.y,
#               xformedPoseMsg.pose.position.z,
#               resRot)
#        return res

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
        try:
            rospy.init_node('robot_scripts', anonymous=True)
        except rospy.ROSException:
            # node already initialized
            pass
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

        if not PR2RobotScript.gearsEngaged:
            return;

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

        if not PR2RobotScript.gearsEngaged:
            return;
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
        if not PR2RobotScript.gearsEngaged:
            return;
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
        if not PR2RobotScript.gearsEngaged:
            return;
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.head.look(pan, tilt, dur=duration)
        if wait:
            PR2RobotScript.waitFor('head_pan_joint', pan);
            PR2RobotScript.waitFor('head_tilt_joint', tilt);
     
        
    @staticmethod
    def rotateHead(newVal, duration=1.0, wait=True):
        if not PR2RobotScript.gearsEngaged:
            return;
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
        if not PR2RobotScript.gearsEngaged:
            return;
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.release(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint', 0.1);
        else:
            PR2RobotScript.waitFor('r_gripper_joint', 0.1);

    @staticmethod
    def closeGripper(side):
        if not PR2RobotScript.gearsEngaged:
            return;
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
        PR2RobotScript.gripper.close(side)
        if side == PR2RobotScript.LEFT:
            PR2RobotScript.waitFor('l_gripper_joint', 0);
        else:
            PR2RobotScript.waitFor('r_gripper_joint', 0);

    @staticmethod
    def setTorso(height, duration=10.0, wait=True):
        if not PR2RobotScript.gearsEngaged:
            return;
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
        if not PR2RobotScript.gearsEngaged:
            return;
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
    def moveBase(place=(0.0,0.0,0.0), rotation=0.0, duration=0.5, wait=True):
        '''
        Move the robot base to a different position, turning its torso at the same time.
        @param place: a tuple with x,y,z coordinates of the destination
        @type place: (float,float,float) 
        @param rotation: simultaneous rotation of the base
        @type rotation: float
        @param duration: duration of motion
        @type duration: float
        '''
        if not PR2RobotScript.gearsEngaged:
            return;
        if not PR2RobotScript.initialized:
            PR2RobotScript.initialize()
  
        motionThread = RobotBaseMotionThread();
        motionThread.start(place[0], place[1], place[2], rotation, duration);
        if wait:
            PR2RobotScript.base.wait_for();
        return;

    @staticmethod
    def pause(duration):
        if not PR2RobotScript.gearsEngaged:
            return;
        rospy.timer.sleep(duration)

    @staticmethod
    def engageGears():
        PR2RobotScript.gearsEngaged = True
        
    @staticmethod
    def disengageGears():
        PR2RobotScript.gearsEngaged = False

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
    
    def start(self, targetX, targetY, targetZ, targetRotDeg, motionDuration):
        if RobotBaseMotionThread.oneThreadRunning is not None:
            #raise RuntimeError("Must stop robot motion thread before starting a new one: Call self.robotMotionThread.stop()");
            RobotBaseMotionThread.oneThreadRunning.stop();
            
        self.targetX = targetX
        self.targetY = targetY
        self.targetZ = targetZ
        self.targetRotDeg = targetRotDeg
        self.motionDuration = motionDuration
                    
        self.motionDuration = motionDuration
        self.keepRunning = True;
        super(RobotBaseMotionThread, self).start();
        
        
    def run(self):
        RobotBaseMotionThread.oneThreadRunning = self;

        properRangeRot = abs(self.targetRotDeg) % 360
        # Take care for target angles > 360:
        if self.targetRotDeg < 0:
            self.targetRotDeg = -properRangeRot
            rotDirection = RotDirection.CLOCKWISE
        else:
             self.targetRotDeg = properRangeRot
             rotDirection = RotDirection.COUNTER_CLOCKWISE

        # Map user input degrees into 0->-180, and -180->0:
        # target degrees positive: 0->180 or 181->360:
        if self.targetRotDeg >= 0 and self.targetRotDeg <= 180:
            targetYawXformed = -self.targetRotDeg
        elif self.targetRotDeg > 180:
            targetYawXformed = 360 - self.targetRotDeg
            
        # target degrees negative: 0->-180 or -181->-360:
        elif self.targetRotDeg < 0 and self.targetRotDeg >= -180:
            targetYawXformed = -self.targetRotDeg
        elif self.targetRotDeg < -181:
            targetYawXformed = 360 + self.targetRotDeg
            
        rotGoalReached = False
        xGoalReached   = False
        yGoalReached   = False
        
        twistMsg  = Twist();
        twistMsg.linear  = Vector3(self.targetX, self.targetY, self.targetZ);
        
        rotSpeedRadPerSec = RobotScript.degree2rad(abs(targetYawXformed)) / self.motionDuration
        if rotDirection == RotDirection.CLOCKWISE:
            rotSpeedRadPerSec = -rotSpeedRadPerSec
        twistMsg.angular  = Vector3(0.0,0.0,rotSpeedRadPerSec);
                        
        # Get initial positions: translation as Vector3 (x,y,z), and rot as quaternion:
        while self.keepRunning and not rospy.is_shutdown():
            try:
                (initialTrans, initialRot) = PR2RobotScript.transformListener.lookupTransform('base_footprint', 'odom_combined', rospy.Time(0));
                
                initialX = initialTrans[0];
                # X axis travel forward runs towards -inf,
                # So must subtract X:
                finalX   = initialX - self.targetX
                
                initialY = initialTrans[1];
                # Y axis: travel left runs towards -inf
                finalY   = initialY - self.targetY
                        
                # Get yaw from rotation quaternion:
                (initRoll, initPitch, initYaw) = euler_from_quaternion(initialRot)
                
                # Map initial rotation from PR2 to user's 0-360 degree:
                initYawDeg = 57.295779504 * initYaw - 1.47009176
                if initYawDeg >= 0:
                    self.currSign = Sign.POSITIVE;
                else:
                    self.currSign = Sign.NEGATIVE;
                
                break;
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
        # Keep whipping the base controller till goals are reached 
        # That whipping happens every UPDATE_PERIOD seconds. At 1.5 times
        # that rate, check location and angle of base to ensure that
        # we don't miss reaching the target:

        valueCheckRate = UPDATE_PERIOD / 1.5
        prevTwistSendTime = rospy.get_time()
        
        while self.keepRunning and not rospy.is_shutdown():
            try:
                # Where is the robot after this iteration's sleep:
                (trans,rot) = PR2RobotScript.transformListener.lookupTransform('base_footprint', 'odom_combined', rospy.Time(0));
                
                newX = trans[0];
                newY = trans[1];
                
                # Current yaw from quaternion:
                (roll,pitch,newYaw) = euler_from_quaternion(rot)
                
                # Map current yaw to user's 0-360: 
                newYawDeg = 57.295779504 * newYaw - 1.47009176
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            # Is rotation goal reached? Yes if target and current rot
            # are of the same sign, and current rotation crossed
            # target angle:
            if  not rotGoalReached and\
                self.angleReachedOrExceeded(targetYawXformed, newYawDeg, rotDirection):
                rotGoalReached = True;
                # No more turning:
                twistMsg.angular = Vector3(0.0,0.0,0.0);
            
            # Are x and y goals reached?
            if not xGoalReached and\
                (
                 ((finalX >= initialX) and (newX >= finalX)) or\
                 ((finalX < initialX) and (newX < finalX))
                 ):
                xGoalReached = True;
                twistMsg.linear = Vector3(0.0, twistMsg.linear.y, 0.0);
            
            if not yGoalReached and\
                (  
                 ((finalY >= initialY) and (newY >= finalY)) or\
                 ((finalY < initialY) and (newY < finalY))
                 ):
                yGoalReached = True;
                twistMsg.linear = Vector3(twistMsg.linear.x, 0.0, 0.0);
            
            if rotGoalReached and xGoalReached and yGoalReached:
                self.stop();
            
            # Time to send a Twist message to keep the base moving?
            timeNow = rospy.get_time();
            if (timeNow - prevTwistSendTime) >= UPDATE_PERIOD:
                PR2RobotScript.baseMovementPublisher.publish(twistMsg);
                prevTwistSendTime = timeNow;
            time.sleep(valueCheckRate)
            continue;           
        
    def stop(self):
        self.keepRunning = False;
        RobotBaseMotionThread.oneThreadRunning = None;
        
    def equalSign(self, a, b):
        return ((a <= 0) and (b <= 0)) or ((a >= 0) and (b >= 0)) 

    def angleReachedOrExceeded(self, targetRot, currRot, rotDirection):
        '''
        Figure out whether robot's rotation has reached or exceeded a
        given desired limit. If 0 rotation is robot looking to 12 o'clock,
        PR2 yaw counter-clockwise runs from 0deg at 12 o'clock to -180deg 
        at 6 o'clock along the left half of the clock face. Yaw continues
        from 6 o'clock being +180deg at 5:20 o'clock to 0deg at noon along
        the right half of the clock face.
         
        @param targetRot: rotation to be reached
        @type targetRot: float (degrees)
        @param currRot: current robot rotation
        @type currRot: float (degrees)
        @param rotDirection: whether robot is turning clockwise or counter-clockwise
        @type rotDirection: RotDirection
        '''
        
        # Did rotation cross a discontinuity? i.e. across
        # 0 or 180?
        if (self.currSign == Sign.POSITIVE) and (currRot < 0):
            # Crossed from positive to negative. Going clockwise
            # that would be from +180 to -180; counter_clockwise
            # it would be +1 to -1
            self.currSign = Sign.NEGATIVE;
            if (rotDirection == RotDirection.CLOCKWISE) and\
                (
                 (targetRot == 180) or\
                 (targetRot == -180)
                 ):
                return True;
            elif (rotDirection == RotDirection.COUNTER_CLOCKWISE) and (targetRot == 0):
                return True;
        elif (self.currSign == Sign.NEGATIVE) and (currRot > 0):
            # Crossed from negative to positive. Going clockwise
            # that would be from -1 to +1; counter_clockwise
            # it would be -180 to +180:
            self.currSign = Sign.POSITIVE;
            if (rotDirection == RotDirection.CLOCKWISE) and (targetRot == 0):
                return True;
            elif (rotDirection == RotDirection.COUNTER_CLOCKWISE) and\
                (
                 (targetRot == 180) or\
                 (targetRot == -180)
                 ):
                return True;
                  
        # No discontinuity crossing: if target and
        # current are in different half-circles, they
        # can't be equal:
        if not self.equalSign(targetRot, currRot):
            return False;
        if rotDirection == RotDirection.COUNTER_CLOCKWISE:
            # are we in left half of circle (0 at top, -180 at bottom):
            if currRot <= 0:
                # Left half of circle, going from 0 to -180
                # Did we overshoot, or are within tolerance just
                # before reaching target?
                if (currRot <= targetRot + Tolerances.BASE_ANGLE):
                    return True;
                else:
                    return False;
            else:
                # Right half of the circle going from 180deg to 0:
                if (currRot <= targetRot + Tolerances.BASE_ANGLE):
                    return True;
                else:
                    return False;
        
        # Going clockwise
        # Are we in left half of circle going -180 toward 0 clockwise?
        if currRot <= 0:
            # Left half going -180 towards 0 clockwise:
            if currRot >= targetRot - Tolerances.BASE_ANGLE:
                return True;
            else:
                return False;
        # Right half of circle going 0 to 180 clockwise:
        if currRot >= targetRot - Tolerances.BASE_ANGLE:
            return True;
        else:
            return False; 


class PR2Base(object):
    
    def wait_for(self):
        while RobotBaseMotionThread.oneThreadRunning is not None:
            time.sleep(0.2);

class RunMotion(threading.Thread):
    
    oneMotionRunning = False
    
    def __init__(self, callable, *args, **kwargs):
        if RunMotion.oneMotionRunning:
            return;
        else:
           RunMotion.oneMotionRunning = True
           
        super(RunMotion, self).__init__();
        self.callable = callable;
        self.args =   args;
        self.kwargs = kwargs;
        self.start();
        
    def start(self, ):
        super(RunMotion, self).start();

    def run(self):
        try:
            if len(self.args) > 0 and len(self.kwargs) > 0: 
                self.callable(self.args, self.kwargs);
            elif len(self.args) > 0 and len(self.kwargs) == 0:
                self.callable(self.args);
            elif len(self.args) == 0 and len(self.kwargs) > 0:
                self.callable(self.kwargs);
            else:
                self.callable();
        except Exception as e:
            rospy.logerr("In RunMotion: " + `e`)
        PR2RobotScript.engageGears();
        RunMotion.oneMotionRunning = False
        
    def stop(self):
        PR2RobotScript.disengageGears();
        
    def pause(self):
        PR2RobotScript.disengageGears();
        
    def resume(self):
        PR2RobotScript.engageGears();

def aboutEq(sensorName, val):
    sensorVal = PR2RobotScript.getSensorReading(sensorName);
    
    if sensorName == "base":
        # val is a Quaternion test all four values:
        if not doComparison(sensorVal.x, val.x, Tolerances.BASE_DISTANCE) or\
           not doComparison(sensorVal.y, val.y, Tolerances.BASE_DISTANCE) or\
           not doComparison(sensorVal.z, val.z, Tolerances.BASE_DISTANCE) or\
           not doComparison(sensorVal.w, val.w, Tolerances.BASE_ANGLE):
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
    
    #rospy.init_node('robot_scripts', anonymous=True)
    #robot = RobotScript()
    #rospy.spin()
    pass
