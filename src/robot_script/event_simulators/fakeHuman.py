#!/usr/bin/env python
import roslib
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')

# Generic libraries
import sys,os,time
from math import *
from random import randint
from collections import OrderedDict;

# ROS libraries
import rospy
from tf import TransformListener, TransformBroadcaster, transformations
from geometry_msgs.msg import *
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import *

# Superclass:
from event_simulator import EventSimulator;

class FakeHuman(EventSimulator):
    "Simulating the perceptual data we would get from a human"
    
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
        super(FakeHuman, self).start(threadSchedule, self.update, repeat=True);

    def publishTFPose(self, pose, name, parent):
        if (pose != None):
            self.tfBroadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                 (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                 rospy.Time.now(), name, parent)

    def getDiff(self, target, current, speed):
    	diff = target - current
    	if diff > speed:
    		return speed, False
    	elif diff < -speed:
    		return -speed, False
    	return diff, True
	 
    def moveTowardsTarget(self, speed):
    	xDiff, xReached = self.getDiff(self.target[0], self.humanPose.position.x, speed[0])
    	yDiff, yReached = self.getDiff(self.target[1], self.humanPose.position.y, speed[1])
    	self.humanPose.position.x += xDiff
    	self.humanPose.position.y += yDiff
    	return (xReached and yReached)

    def update(self, dummy):
        '''
        Called repeatedly from the EventSimulator thread. Moves the Rviz human towards
        various locations in small increments. Locations, speed, and move times are
        taken from self.motionSchedule, which was passed into the start() method.
        Example schedule is the following dict (keys are fractional seconds): 
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
        # Move a little bit:
        self.moveTowardsTarget(keyframeSpeed);
        self.visualizeFakeHuman();


    def visualizeFakeHuman(self):
        # Visualize the fake human:
        if (self.humanPose != None):
            self.publishTFPose(self.humanPose, 'fake_human', 'odom_combined')
        
        m = Marker(type=Marker.SPHERE, id=0, lifetime=rospy.Duration(2), pose=self.humanPose,
                    scale=Vector3(0.3,0.3,0.3), header=Header(frame_id='odom_combined'),
                    color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        self.markerPublisher.publish(m)
            

if __name__ == "__main__":

    human = FakeHuman()
    motionSchedule = OrderedDict();
    motionSchedule[2]  = None;
    motionSchedule[5]  = {'target': [2,-1], 'speed': [0.1,0.03]}; 
    motionSchedule[10] = None;
    motionSchedule[12] = {'target': [4,-2], 'speed': [.1,.055]}; 
    motionSchedule[16] = None;
    motionSchedule[20] = {'target': [0.6,.5], 'speed': [.08,.1]};
    motionSchedule[27] = None;    
    
    #rospy.spin()
#    while(not rospy.is_shutdown()):
#        human.update()
    human.start(motionSchedule);
    time.sleep(0.1)

