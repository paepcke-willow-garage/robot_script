#!/usr/bin/env python
import roslib
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')

# Generic libraries
import sys,os,time
import numpy
from math import *
from random import randint
from collections import OrderedDict;
import signal;

# ROS libraries
import rospy
from tf import TransformListener, TransformBroadcaster, transformations
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *

# Superclass:
from event_simulator import EventSimulator;

# Time between invoking callback to client's script
# (such as testFakeHuman.py)
FRAME_INTERVAL = 0.1; # seconds

# Enum:
class MotionDirection:
    AWAY_FROM_ROBOT = 0
    TOWARDS_ROBOT = 1


class FakeHuman(EventSimulator):
    '''
    Simulating the perceptual data we would get from a human.
    The human is symbolized by a red ball at roughly human size
    relative to the robot in the view.
    
    Given a schedule of poses for the human, instances of
    this class move the human through the scheduled timed poses. 
    A schedule looks like this:
    
		motionSchedule = OrderedDict();
		
		motionSchedule[3]  = {'target': [2,0], 'speed': [0.5,0.0]}; 
		motionSchedule[6]  = {'target': [3,0], 'speed': [0.5,0.0]}; 
		motionSchedule[9]  = {'target': [4,0], 'speed': [0.5,0.0]};
    
    Here the motionSchedule keys (3,6, and 9) are seconds from program start.
    the 'target' numbers are meters of distance from the robot, and the 
    two speed components are meters/sec in the x and y direction, respectively.
    
    When the scheduled pose sequence is completed, it starts over.
    Start the sequence with a call to a FakeHuman's start() method, passing
    it a schedule. Stop the motions via a call to the stop() method.
    
    '''
    
    
    
    def __init__(self):
        self.tfBroadcaster = TransformBroadcaster()
        q = transformations.quaternion_from_euler(0,0,pi)
        self.humanPose = Pose(Point(5, 0, 1.65), Quaternion(*q))
        self.markerPublisher = rospy.Publisher('visualization_marker', Marker)
        rospy.loginfo('Initialized.')
        self.startTime = time.time()
        rospy.init_node('fake_human_node', anonymous=True)

    def start(self, motionSchedule):

        self.motionSchedule = motionSchedule;
        self.motionScheduleKeys = motionSchedule.keys();
        # Current index into the motion schedule keys:
        self.motionScheduleIndex = 0;
        
        threadSchedule = OrderedDict();
        # Create a schedule that calls the callback function (i.e. self.update())
        # at 0.2 seconds, and then repeats. The None is the argument passed
        # to self.update().
        threadSchedule[0.2] = None;
        self.startTime = time.time();
        #******super(FakeHuman, self).start(threadSchedule, self.update, repeat=True, callbackInterval=FRAME_INTERVAL);
        super(FakeHuman, self).start(motionSchedule, self.update, repeat=True, callbackInterval=FRAME_INTERVAL);

    def publishTFPose(self, pose, name, parent):
        if (pose != None):
            self.tfBroadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                 (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                 rospy.Time.now(), name, parent)

    def getPlanarDistance(self, pose):
        '''
        Compute distance of a given pose from its origin in a 2d plane.
        @param pose: pose to examine
        @type pose: Pose
        @return: linear distance
        @rType: float
        '''
        vec = numpy.array((pose.position.x, pose.position.y))
        self.planarDistance = numpy.linalg.norm(vec) 
        return self.planarDistance

    def getDiff(self, target, current, speed):
    	diff = target - current
    	if diff > speed:
    		return speed, False
    	elif diff < -speed:
    		return -speed, False
    	return diff, True
	 
    def moveTowardsTarget(self, speed):
        '''
        Called from update(). Moves the rviz representation of the human towards 
        a target position.
        @param speed: motion speed
        @type speed: float
        @return: whether fake human moved closer to the robot, or away from it
        @rtype: MotionDirection
        '''
        initialDistanceFromRobot = self.getPlanarDistance(self.humanPose);
    	xDiff, xReached = self.getDiff(self.target[0], self.humanPose.position.x, speed[0])
    	yDiff, yReached = self.getDiff(self.target[1], self.humanPose.position.y, speed[1])
    	self.humanPose.position.x += xDiff
    	self.humanPose.position.y += yDiff
        newDistanceFromRobot = self.getPlanarDistance(self.humanPose)
        if (newDistanceFromRobot - initialDistanceFromRobot) > 0:
            return MotionDirection.TOWARDS_ROBOT
        else:
            return MotionDirection.AWAY_FROM_ROBOT

    def update(self, dummy):
        '''
        Called repeatedly from the EventSimulator thread. Moves the Rviz human towards
        various locations in small increments. Locations, speed, and move times are
        taken from self.motionSchedule, which was passed into the start() method.
        Example schedule is the following dict (keys are fractional seconds):: 
        
                motionSchedule[2]  = None;
                motionSchedule[5]  = {target: [2,-1], speed: [0.1,0.03]}; 
                motionSchedule[10] = None;
                motionSchedule[12] = {target: [4,-2], speed: [.1,.055]}; 
                motionSchedule[16] = None;
                motionSchedule[20] = {target: [0.6,.5], speed: [.08,.1]};
                motionSchedule[27] = None;    
		
        @param dummy: EventSimulator calls with an argument, which is None in our case, and unused.
        @type dummy: None.
        '''

        timePassed = time.time() - self.startTime;        
        
        # Get current motion's upper time bound: 
        keyframeTime   = self.motionScheduleKeys[self.motionScheduleIndex];
        # Is time later than that end time of current motion?
        while timePassed >= keyframeTime:
            # Grab the next schedule entry:
            self.motionScheduleIndex += 1;
            if self.motionScheduleIndex >= len(self.motionSchedule):
                # Ran out of schedule entries. Start over:
                self.motionScheduleIndex = 0;
                self.startTime = time.time();
                timePassed = time.time() - self.startTime;
                break;
            else:
                # Grab the new schedule entry's upper time bound (the dict key):
                keyframeTime = self.motionScheduleKeys[self.motionScheduleIndex];
                
        try:
            # Get motion x/y and x/y speeds:
            keyframeTarget = self.motionSchedule[keyframeTime]['target'];
            keyframeSpeed  = self.motionSchedule[keyframeTime]['speed'];
        except TypeError:
            # Motion schedule value for this entry is None; do nothing:
            self.visualizeFakeHuman();
            return;
        
        self.target = keyframeTarget;
        targetPose  = Pose(Point(keyframeTarget[0],keyframeTarget[1],0), Quaternion(0,0,0,0))
        self.targetDistance = self.getPlanarDistance(targetPose);
        #******************** 
        #print(str(timePassed) + "s. Target: " + str(self.targetDistance) + "m. CurrDist: " + str(self.getPlanarDistance(self.humanPose)));
        #********************         
        # Move the human a little bit (we ignore the move direction):
        motionDirection = self.moveTowardsTarget(keyframeSpeed);
        self.visualizeFakeHuman();
        # If the human reached a keyframe destination,
        # return the new xy to the simulator, which will put it
        # into its out queue: 
        newDistance = self.getPlanarDistance(self.humanPose);
        return newDistance;
        
    def visualizeFakeHuman(self):
        # Visualize the fake human:
        if (self.humanPose != None):
            self.publishTFPose(self.humanPose, 'fake_human', 'odom_combined')
        
        m = Marker(type=Marker.SPHERE, id=0, lifetime=rospy.Duration(2), pose=self.humanPose,
                    scale=Vector3(0.3,0.3,0.3), header=Header(frame_id='odom_combined'),
                    color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        self.markerPublisher.publish(m)
      
def sigintHandler(signum, frame):
    human.stop();      
signal.signal(signal.SIGINT, sigintHandler);

if __name__ == "__main__":

# ----------------------------------------  For Testing Only. Scripts Call the start() Method ------------------
    human = FakeHuman()
    motionSchedule = OrderedDict();
#    motionSchedule[2]  = None;
#    motionSchedule[5]  = {'target': [2,-1], 'speed': [0.1,0.03]}; 
#    motionSchedule[10] = None;
#    motionSchedule[12] = {'target': [4,-2], 'speed': [.1,.055]}; 
#    motionSchedule[16] = None;
#    motionSchedule[20] = {'target': [0.6,.5], 'speed': [.08,.1]};
#    motionSchedule[27] = None;    
    
    motionSchedule[3]  = {'target': [2,0], 'speed': [0.1,0.0]}; 
    motionSchedule[6]  = {'target': [3,0], 'speed': [0.1,0.0]}; 
    motionSchedule[9]  = {'target': [4,0], 'speed': [0.1,0.0]};
    
    human.start(motionSchedule);
    time.sleep(0.1)

