#!/usr/bin/env python

'''
pr2.openGripper(RIGHT)
pr2.closeGripper(LEFT)
pr2.tiltHead(-30);
pr2.rotateHead(30);
'''


#from robot_scripting import RobotScript
#from robot_scripting import RobotScript as rs


from robot_scripting import PR2RobotScript as pr2
from robot_scripting import equallish

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

if pr2.getSensorReading("l_gripper_joint") |equallish| 0:
    pr2.openGripper(LEFT)
else:
    pr2.closeGripper(LEFT)   

if pr2.getSensorReading("r_gripper_joint") |equallish| 0:
    pr2.openGripper(RIGHT)
else:
    pr2.closeGripper(RIGHT)   

print pr2.getSensorReading('head_tilt_joint')

if pr2.getSensorReading('head_tilt_joint') |equallish| 30:
    pr2.tiltHead(-30);
else:
    pr2.tiltHead(30);
    
#pr2.rotateHead(30);
