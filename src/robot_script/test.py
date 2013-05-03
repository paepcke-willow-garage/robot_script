#!/usr/bin/env python

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq
import rospy
import sys

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

#if aboutEq('torso_lift_joint', 0):
#    pr2.setTorso(10)
#else:
#    pr2.setTorso(0)

if aboutEq('head_tilt_joint', 30):
    pr2.lookAt(-100, -20 , 5, wait=False)
else:
    pr2.lookAt(+100, +30 , 5, wait=False)

#if aboutEq('head_tilt_joint', 30):
#    #pr2.tiltHead(-20,5, wait=True)
#    pr2.tiltHead(-20,5, wait=False)
##    pr2.panHead(-100,5, wait=False)
#    print "Head tilt to -20 done;"
#else:
#    #pr2.tiltHead(+30,5, wait=True)
#    pr2.tiltHead(+30,5, wait=False)
##    pr2.panHead(+100,5, wait=False)
#    print "Head tilt to +30 done;"
#for i in range(40):
#    print str(pr2.getSensorReading('head_tilt_joint'))

    


