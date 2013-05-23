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
    
    motionSchedule[15]   = {'target': [1,0], 'speed': [0.05,0.0]} 
    motionSchedule[20]  = {'target': [3,0], 'speed': [0.05,0.0]} 
    motionSchedule[25]  = {'target': [4,0], 'speed': [0.05,0.0]}
    
    # When the scheduled pose sequence is completed, it starts over.
    # Start the sequence with a call to a FakeHuman's start() method, passing
    # it a schedule. Stop the motions via a call to the stop() method:
        
    human.start(motionSchedule)


#    #intial Pose robot starts in
#    pr2.closeGripper(LEFT)
#    pr2.closeGripper(RIGHT)
#    pr2.moveBase(rotation=0, duration=0.5)    
#    pr2.tiltHead(20,2, wait=False)
#    pr2.rotateHead(-45, 2, wait=False)
#    joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
#    values = [10,-125,70,90,-70,15, 0]
#    pr2.moveArmJoint(joints,values, duration=2.0, wait=False)
#    joints = ['r_shoulder_pan_joint','r_elbow_flex_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint','r_wrist_flex_joint','r_upper_arm_roll_joint', 'r_wrist_roll_joint']
#    values = [-10,-125,70,-90,-70,-15, 0]
#    pr2.moveArmJoint(joints,values, duration=2.0)
#    pr2.setTorso(.02, 2.0)
#    rospy.timer.sleep(5)

def sequence1(animation):
    '''
    This function runs the robot through some motion forever.
    It doesn't need to. It could just be a long-running motion
    which does eventually stop.
    '''
    pr2.openGripper(LEFT)
    pr2.panHead(-60)
    pr2.moveArmJoint('l_shoulder_pan_joint', 0, duration=2.0)            
    pr2.moveArmJoint('l_shoulder_pan_joint', 90, duration=2.0)
    pr2.panHead(0)
    pr2.closeGripper(LEFT)

def sleep_while_running(duration, animation):
    start = time.time()
    is_timedout = False
    while (not is_timedout and not animation.is_stopped):
        is_timedout = (time.time()-start) > duration
        time.sleep(0.05)

def sequence2(animation):
    #Robot's reaction to someone entering his personal space to interact with him.
    print 'starting sequence 2'
    pr2.rotateHead(human_angle, 2,wait=False)
    pr2.setTorso(.04 + pr2.getSensorReading("torso_lift_joint"), 1.0,  wait=False)
    #pr2.moveBase(rotation=new_PS_Angle, duration=7.0)
    joints = ['r_forearm_roll_joint', 'r_shoulder_lift_joint','r_wrist_roll_joint', 'r_wrist_flex_joint', 'r_upper_arm_roll_joint','r_shoulder_pan_joint'  ]
    value = [-90, 40, 90, 0, 0, 0]
    pr2.moveArmJoint(joints, value, duration=2.0, wait=False)
    joints = ['l_forearm_roll_joint', 'l_shoulder_lift_joint','l_wrist_roll_joint', 'l_wrist_flex_joint', 'l_upper_arm_roll_joint','l_shoulder_pan_joint'  ]
    value = [60, 70, 0, -80, 0, 15]
    pr2.moveArmJoint(joints, value, duration=2.0)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=1)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=1.25)
    pr2.moveArmJoint(['r_upper_arm_roll_joint'], [0], duration=1.5)

    sleep_while_running(7, animation)
    
    pr2.rotateHead(2, .5)

    sleep_while_running(2, animation)
    
    pr2.tiltHead(-5, .5)
    
    sleep_while_running(6, animation)
    
    pr2.rotateHead(0, .5)
    
    sleep_while_running(14, animation)
    
    pr2.setTorso(-.01 + pr2.getSensorReading("torso_lift_joint"), 4.0)
    joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
    values = [10,-125,70,90,-70,15, 0]
    
    pr2.moveArmJoint(joints,values, duration=3.0, wait=False)
    joints = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint','l_upper_arm_roll_joint']
    values = [human_angle *.2 , 70, 0]
    pr2.moveArmJoint(joints,values, duration=3.0)
    
    print 'Returning from sequence 2'
    #rospy.timer.sleep(800)
    
def sequence3(animation):
    #a distance robot will notice someone.
    print 'starting sequence 3'
    pr2.tiltHead(-8,1)
    pr2.rotateHead(human_angle, 1.5)
    #pr2.setTorso(.01, 4)
    print 'return sequence 3'
    
def sequence4(animation):
    print 'starting sequence 4'
    pr2.setTorso(.02, 4)
    rospy.timer.sleep(3)
    print 'return sequence 4'


human_angle = 0
animation1 = Motion(sequence1)
animation2 = Motion(sequence2)
animation3 = Motion(sequence3)
animation4 = Motion(sequence4)

def main():
    __init__()

    
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
    
    
    while (True):
        # Wait for a new position report from the human:
        
        newHumanLocation = eventQueue.get()
        human_angle = 0 #TODO: update angle based on new loc
        
        #print('Human now at ' + str(newHumanLocation))
        
        ## UPDATE THE STATE
        
        if state == NO_HUMANS:
            ## In NO_HUMANS 
    
    
            if newHumanLocation < 4:
                state = VISIBLE_HUMANS
                print('Current state ' + state)
                ## NO_HUMANS --> VISIBLE_HUMANS
                animation4.stop()
                animation3.start()
        
        elif state == VISIBLE_HUMANS:
            ## In CLOSE_HUMANS 
    
            
            if newHumanLocation < 2:
                state = CLOSE_HUMANS
                print('Current state ' + state)
                ## VISIBLE_HUMANS --> CLOSE_HUMANS
                animation3.stop()
                animation2.start()
                
            elif newHumanLocation > 4.5:
                state = NO_HUMANS
                print('Current state ' + state)
                ## VISIBLE_HUMANS --> NO_HUMANS
                animation3.stop()
                animation4.start()
        
        elif state == CLOSE_HUMANS:
            ## In CLOSE_HUMANS 
            
            
            if newHumanLocation > 2.5:
                state = VISIBLE_HUMANS
                print('Current state ' + state)
                
                ## CLOSE_HUMANS --> VISIBLE_HUMANS
                animation2.stop()
                animation3.start()
        else:
            print 'ERROR: Unknown state.'
    
main()  





