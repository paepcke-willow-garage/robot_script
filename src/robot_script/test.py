#!/usr/bin/env python

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq
from geometry_msgs.msg import Quaternion
import rospy
import sys
from robot_script.robot_scripting import PR2RobotScript

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

#if aboutEq('torso_lift_joint', 0):
#    pr2.setTorso(10)
#else:
#    pr2.setTorso(0)

#if aboutEq('head_tilt_joint', 30):
#    pr2.lookAt(-100, -20 , 5, wait=False)
#else:
#    pr2.lookAt(+100, +30 , 5, wait=False)

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

#print(str(pr2.odometryFromBaseFrame(0.0, 0.0, 0.0, 90)))


#pr2.moveBase(rotation=10)
#pr2.moveBase(rotation=0)        
#pr2.moveBase(rotation=90)
#pr2.moveBase(rotation=-90)
#pr2.moveBase(rotation=359)
#pr2.moveBase(rotation=135)
#pr2.moveBase(rotation=-135)

#
#pr2.moveBase(place=(1.0,0.0,0.0))    
#pr2.moveBase(place=(-1.0,0.0,0.0))    
#pr2.moveBase(place=(0.0,1.0,0.0))    
#pr2.moveBase(place=(0.0,-1.0,0.0))    

pr2.moveBase(place=(1.0,-3.0,0.0), rotation=135)    
#pr2.moveBase(place=(1.0,-3.0,0.0))
#pr2.moveBase(rotation=135)    



#from tf.transformations import euler_from_quaternion
#from geometry_msgs.msg import Quaternion
#
#for deg in range(360):
#    rad = pr2.degree2rad(deg)
#    quat = (0.0,0.0,1.0,rad)
#    (roll,pitch,yaw) = euler_from_quaternion(quat)
#    print("Yaw " + str(deg) + " deg (" + str(rad) + "rad): " + str(yaw))
