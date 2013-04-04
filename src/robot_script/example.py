#!/usr/bin/env python

#from robot_scripting import RobotScript
#from robot_scripting import RobotScript as rs


from robot_scripting import PR2RobotScript as pr2

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

pr2.openGripper(LEFT)
pr2.openGripper(RIGHT)
#pr2.closeGripper(LEFT)
#pr2.closeGripper(RIGHT)

