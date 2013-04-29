#!/usr/bin/env python

'''
List of available commands:
---------------------------

Gripper and Head:
    pr2.openGripper(side, wait=True)    # side is LEFT or RIGHT
    pr2.closeGripper(side, wait=True)   # side is LEFT or RIGHT
    pr2.tiltHead(-30, wait=True)        # can add something like 'duration=2.0' for motion duration 
    pr2.rotateHead(30, wait=True)       # can add something like 'duration=2.0' for motion duration 
 
Moving the arm joints one at a time, or together:
    pr2.moveArmJoint(jointName, newAngle, duration=2.0, wait=False)
    pr2.moveArmJoint([jointName1, jointName2, ...], [newAngle1, newAngle2, ...], duration=2.0, wait=True)

Moving the whole robot (the base):
    pr2.moveBase(place=(x,y,z), rotation=deg, duration=2, wait=True)
 
Reading joint values:
    pr2.getSensorReading(sensorName)

Checking approximate joint value (when setting a joint to 0, it's often something like 0.0000134; aboutEq() does an approximate compare)
    aboutEq(jointName, value)

Raising/lowering the torso:
    pr2.setTorso(.05, 2.0, wait=True)

Temporarily pausing execution (but already initiated robot motions continue): 
    rospy.timer.sleep(seconds)

More examples:
	pr2.tiltHead(15 + pr2.getSensorReading("head_tilt_joint"), 1)
	
	joints = ['l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint']
	values = [-70, -90, -70]
	pr2.moveArmJoint(joints, values, duration=2.0, wait=False)

'''

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq
from robot_scripting import FullPose

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

if aboutEq("l_gripper_joint", 0):
    pr2.openGripper(LEFT)
else:
    pr2.closeGripper(LEFT)   
if aboutEq("r_gripper_joint", 0):
    pr2.openGripper(RIGHT)
else:
    pr2.closeGripper(RIGHT)   

if aboutEq('head_tilt_joint', 30):
    pr2.tiltHead(-30, duration=0.5)
else:
    pr2.tiltHead(30, duration=0.5)
    
if aboutEq('head_pan_joint', 60):
    pr2.panHead(-60)
else:
    pr2.panHead(60)

if aboutEq('l_shoulder_pan_joint', 90):
    pr2.moveArmJoint('l_shoulder_pan_joint', 0, duration=2.0)
    pr2.moveArmJoint(['l_forearm_roll_joint', 'l_elbow_flex_joint'], [30, 0], duration=2.0)
else:
    pr2.moveArmJoint('l_shoulder_pan_joint', 90, duration=2.0)
    pr2.moveArmJoint(['l_forearm_roll_joint', 'l_elbow_flex_joint'], [-30, -130], duration=2.0)    

pr2.moveBase(place=(0.3,0.0,0.0))
pr2.moveBase(place=(-0.3,0.0,0.0))
pr2.moveBase(rotation=90)
pr2.moveBase(rotation=-90)

torsoState = pr2.getSensorReading('torso_lift_joint')

if aboutEq('torso_lift_joint', 0):
    pr2.setTorso(0.3, wait=True)
    print "Torso done"
else:
    pr2.setTorso(0, duration=5, wait=True)
    print "Torso done"

