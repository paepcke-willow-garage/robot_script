#!/usr/bin/env python

'''
List of available commands:
---------------------------

Gripper and Head:
    pr2.openGripper(side)    # side is LEFT or RIGHT
    pr2.closeGripper(side)   # side is LEFT or RIGHT
    pr2.tiltHead(-30)        # can add something like 'duration=2.0' for motion duration 
    pr2.rotateHead(30)       # can add something like 'duration=2.0' for motion duration 
 
Moving the arm joints one at a time, or together:
    pr2.moveArmJoint(jointName, newAngle, duration=2.0, wait=False)
    pr2.moveArmJoint([jointName1, jointName2, ...], [newAngle1, newAngle2, ...], duration=2.0, wait=True)
    pr2.moveBase(place=(x,y,z), rotation=deg, duration=2) #
 
Waiting for a motion to complete:
    pr2.waitFor(jointName, duration)

Reading joint values:
    pr2.getSensorReading(sensorName)

Checking approximate joint value (when setting a joint to 0, it's often something like 0.0000134; aboutEq() does an approximate compare)
    aboutEq(jointName, value)

Raising/lowering the torso:
    pr2.setTorso(.05, 2.0)

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
import rospy
import sys

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

if aboutEq('head_tilt_joint', 30):
    #pr2.tiltHead(-20,5, wait=True)
    pr2.tiltHead(-20,5, wait=False)
    print "Head tilt to -20 done;"
else:
    #pr2.tiltHead(+30,5, wait=True)
    pr2.tiltHead(+30,5, wait=False)
    print "Head tilt to +30 done;"
for i in range(40):
    print str(pr2.getSensorReading('head_tilt_joint'))

    


