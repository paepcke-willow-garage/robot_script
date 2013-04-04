#!/usr/bin/env python

'''
pr2.openGripper(RIGHT)
pr2.closeGripper(LEFT)
pr2.tiltHead(-30)
pr2.rotateHead(30)

pr2.waitFor(jointName)

pr2.getSensorReading(sensorName)

aboutEq(jointName, value)

'''

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq

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

