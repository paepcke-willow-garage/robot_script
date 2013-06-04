#!/usr/bin/env python

# This example script illustrates how robot scripts may 
# use the fake-human event simulator. A ball approaches,
# and retreats from the robot according to a specified
# schedule. 

# Before you start this script, quit Gazebo, and type this
# into a terminal:
#
#    roslaunch robot_script fakeHumanExampleNeededServices.launch

import signal
import rospy
import time
import sys
from robot_scripting import PR2RobotScript as pr2
from robot_scripting import Motion
from robot_scripting import aboutEq
from event_simulators.event_simulator import EventSimulator
from Queue import Empty
from collections import OrderedDict
from event_simulators.fakeHuman import FakeHuman 

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

human = FakeHuman()

def __init__():

    # Create a schedule:
    motionSchedule = OrderedDict()
    
    
    # In the following motionSchedule keys (3,6, and 9) are seconds from program start.
    # the 'target' numbers are meters of distance from the robot, and the 
    # two speed components are meters/sec in the x and y direction, respectively.
    
    motionSchedule[12]   = {'target': [6,0], 'speed': [0.05,0.0]}
    motionSchedule[22]   = {'target': [1,-1], 'speed': [0.05,0.0]}
    motionSchedule[30]  = {'target': [3,2], 'speed': [0.05,0.0]}
    motionSchedule[50]  = {'target': [4,0], 'speed': [0.05,0.0]}
    
    # When the scheduled pose sequence is completed, it starts over.
    # Start the sequence with a call to a FakeHuman's start() method, passing
    # it a schedule. Stop the motions via a call to the stop() method:
        
    human.start(motionSchedule)



def defaultPose(animation):
    rospy.loginfo('Starting Default Pose')
    pr2.closeGripper(LEFT)
    rospy.loginfo('close 1')
    pr2.closeGripper(RIGHT)
    rospy.loginfo('close 2')
    pr2.moveBase(rotation=0, duration=0.5)    
    rospy.loginfo('moved base')
    pr2.tiltHead(20,2, wait=False)
    rospy.loginfo('tilted')
    pr2.rotateHead(-25, 2, wait=False)
    joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
    values = [10,-125,70,90,-70,15, 0]
    pr2.moveArmJoint(joints,values, duration=2.0, wait=False)
    joints = ['r_shoulder_pan_joint','r_elbow_flex_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint','r_wrist_flex_joint','r_upper_arm_roll_joint', 'r_wrist_roll_joint']
    values = [-10,-125,70,-90,-70,-15, 0]
    pr2.moveArmJoint(joints,values, duration=2.0)
    pr2.setTorso(.02, 2.0)
    animation.sleep_while_running(5)
    rospy.loginfo('End of Default Pose')
  
def headKeepAlive(animation, motionPauseSecs):
    rospy.loginfo('Starting Head Keep Alive')
    #pr2.rotateHead(pr2.getSensorReading("head_pan_joint") +5, .5)
    #animation.sleep_while_running(1)
    #pr2.tiltHead(pr2.getSensorReading("head_tilt_joint") -5, .85)
    #animation.sleep_while_running(3)
    pr2.rotateHead(25, 1)
    animation.sleep_while_running(motionPauseSecs)
    pr2.rotateHead(-25, 1)
    animation.sleep_while_running(motionPauseSecs)
    rospy.loginfo('End of Head Keep Alive')


def sequence1(animation):
    
    #This function runs the robot through some motion forever.
    #It doesn't need to. It could just be a long-running motion
    #which does eventually stop.
    
    rospy.loginfo('Starting sequence 1')
    pr2.setTorso(.02, 4)
    animation.sleep_while_running(6)
    rospy.loginfo('End of sequence 1')

def sequence2(animation):
    #Robot's reaction to someone entering his personal space to interact with him.
    rospy.loginfo('Starting sequence 2')

    pr2.rotateHead(humanAngle, 2,wait=False)
    pr2.setTorso(.04 + pr2.getSensorReading("torso_lift_joint"), 1.0,  wait=False)
    #pr2.moveBase(rotation=new_PS_Angle, duration=7.0)
    joints = ['r_forearm_roll_joint', 'r_shoulder_lift_joint','r_wrist_roll_joint', 'r_wrist_flex_joint', 'r_upper_arm_roll_joint','r_shoulder_pan_joint'  ]
    value = [60, 40, 90, 0, 0, 0]
    pr2.moveArmJoint(joints, value, duration=2.0, wait=False)
    joints = ['l_forearm_roll_joint', 'l_shoulder_lift_joint','l_wrist_roll_joint', 'l_wrist_flex_joint', 'l_upper_arm_roll_joint','l_shoulder_pan_joint'  ]
    value = [60, 70, 0, -80, 0, 15]
    pr2.moveArmJoint(joints, value, duration=2.0)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=1.25)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [0], duration=1.5)

    animation.sleep_while_running(7)
    pr2.rotateHead(2, .5)
    animation.sleep_while_running(2)
    pr2.tiltHead(-5, .5)
    animation.sleep_while_running(6)
    pr2.rotateHead(0, .5)
    animation.sleep_while_running(14)
    
    pr2.setTorso(-.01 + pr2.getSensorReading("torso_lift_joint"), 4.0)
    joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
    values = [10,-125,70,90,-70,15, 0]
    
    pr2.moveArmJoint(joints,values, duration=3.0, wait=False)
    joints = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint','l_upper_arm_roll_joint']
    values = [humanAngle *.2 , 70, 0]
    pr2.moveArmJoint(joints,values, duration=3.0)
    
    rospy.loginfo('End of sequence 2')
    #animation.sleep_while_running(800)
    
def sequence3(animation):
    #a distance robot will notice someone.
    rospy.loginfo('Starting sequence 3')
    pr2.tiltHead(-8,1)
    pr2.rotateHead(humanAngle, 1.5)
    #pr2.setTorso(.01, 4)
    rospy.loginfo('End of sequence 3')
    
def sequence4(animation):
    rospy.loginfo('Starting sequence 4')
    pr2.setTorso(.02, 4)
    animation.sleep_while_running(3)
    rospy.loginfo('End of sequence 4')


humanAngle = 0
animation1 = Motion(sequence1)
animation2 = Motion(sequence2)
animation3 = Motion(sequence1)
animation4 = Motion(sequence4)

headKeepAliveAnim = Motion(headKeepAlive)
defaulPoseAnim = Motion(defaultPose)

def main():
    __init__()
       
    #HeadKeepAliveAnim.start()
    #rospy.timer.sleep(45)
    #HeadKeepAliveAnim.stop()
    
    
    # Obtain a queue from which human distances are obtained as
    # they change:
    eventQueue = human.getEventQueue()
    
    # Motions, like the function "longMotionSequence" above are 
    # started using the following statement. Note that you 
    # can currently only have one motion sequence running at
    # the same time. You can stop a motion anytime by calling
    # myMotion.stop(). For pausing a motion, see below:
    
    
    NO_HUMANS = 'NO_HUMANS'
    VISIBLE_HUMANS = 'VISIBLE_HUMANS'
    CLOSE_HUMANS = 'CLOSE_HUMANS'
    
    state = NO_HUMANS
    
    defaulPoseAnim.start()
    while not defaulPoseAnim.is_done():
        time.sleep(0.01)
    defaulPoseAnim.stop()
    
    #headKeepAlive(None)
    # Two seconds between the two head motions:
    # This is an example for a parallel function
    # taking an argument:
    
    #headKeepAliveAnim.start(5)
    
    while (True):
        # Wait for a new position report from the human:
        
        newHumanLocation = eventQueue.get()
        humanDistance = pr2.getPlanarDistance(newHumanLocation)
        humanAngle = pr2.getAngle(newHumanLocation)
        
        pr2.displayInfo('Robot state:' + state)
        
        ## UPDATE THE STATE
        
        if state == NO_HUMANS:
            ## In NO_HUMANS 
            pr2.lookAtPoint(newHumanLocation, 1.0, wait=False)
    
            if humanDistance < 4:
                state = VISIBLE_HUMANS
                print('Current state ' + state)
                ## NO_HUMANS --> VISIBLE_HUMANS
                animation4.stop()
                animation3.start()
        
        elif state == VISIBLE_HUMANS:
            ## In CLOSE_HUMANS
            pr2.lookAtPoint(newHumanLocation, 1.0, wait=False)
            
            if humanDistance < 2:
                state = CLOSE_HUMANS
                print('Current state ' + state)
                ## VISIBLE_HUMANS --> CLOSE_HUMANS
                animation3.stop()
                animation2.start()
                
            elif humanDistance > 4.5:
                state = NO_HUMANS
                print('Current state ' + state)
                ## VISIBLE_HUMANS --> NO_HUMANS
                animation3.stop()
                animation4.start()
        
        elif state == CLOSE_HUMANS:
            ## In CLOSE_HUMANS 
            
            
            if humanDistance > 2.5:
                state = VISIBLE_HUMANS
                print('Current state ' + state)
                
                ## CLOSE_HUMANS --> VISIBLE_HUMANS
                animation2.stop()
                animation3.start()
        else:
            print 'ERROR: Unknown state.'
    
main()  





