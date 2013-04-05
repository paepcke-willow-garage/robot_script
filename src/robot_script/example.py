#!/usr/bin/env python

'''
List of available commands:
pr2.openGripper(side)    # side is LEFT or RIGHT
pr2.closeGripper(side)   # side is LEFT or RIGHT
pr2.tiltHead(-30)        # can add something like 'duration=2.0' for motion duration 
pr2.rotateHead(30)       # can add something like 'duration=2.0' for motion duration 
pr2.moveArmJoint(jointName, newAngle, duration=2.0)
pr2.moveBase(fullPose)   # can add something like 'duration=2.0' for motion duration 

fullPose = FullPose(place=(0.3,0.0,0.0), angle=90)

pr2.waitFor(jointName)

pr2.getSensorReading(sensorName)

aboutEq(jointName, value)

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
else:
    pr2.moveArmJoint('l_shoulder_pan_joint', 90, duration=2.0)

newBasePose = FullPose(place=(0.3,0.0,0.0))
pr2.moveBase(newBasePose)

newBasePose = FullPose(place=(-0.3,0.0,0.0))
pr2.moveBase(newBasePose)

newBasePose = FullPose(rotation=90)
pr2.moveBase(newBasePose)
newBasePose = FullPose(rotation=-90)
pr2.moveBase(newBasePose)

torsoState = pr2.getSensorReading('torso_lift_joint')

if aboutEq('torso_lift_joint', 0):
    pr2.setTorso(10)
    pr2.waitFor('torso_lift_joint')
else:
    pr2.setTorso(0, duration=5)
    pr2.waitFor('torso_lift_joint')

