#!/usr/bin/env python

'''
List of available commands:
pr2.openGripper(side)    # side is LEFT or RIGHT
pr2.closeGripper(side)   # side is LEFT or RIGHT
pr2.tiltHead(-30)        # can add something like 'duration=2.0' for motion duration 
pr2.rotateHead(30)       # can add something like 'duration=2.0' for motion duration 

pr2.moveArmJoint(joints, Values, duration=2.0)
pr2.moveArmJoint(jointName, newAngle, duration=2.0, wait=False)
pr2.moveArmJoint([jointName1, jointName2, ...], [newAngle1, newAngle2, ...], duration=2.0, wait=True)
pr2.moveBase(place=(x,y,z), rotation=deg, duration=2) #

pr2.waitFor(jointName, duration)

pr2.getSensorReading(sensorName)

pr2.setTorso(.05, 2.0)

rospy.timer.sleep(seconds)

pr2.tiltHead(15 + pr2.getSensorReading("head_tilt_joint"), 1)

joints = ['l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint']
values = [-70, -90, -70]
pr2.moveArmJoint(joints, values, duration=2.0, wait=False)

aboutEq(jointName, value)

'''

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq
from robot_scripting import FullPose
import rospy

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH


def __init__():

    pr2.closeGripper(LEFT)
    pr2.closeGripper(RIGHT)
    pr2.moveBase(place=(0.3,0.0,0.0), rotation=-10)    
    pr2.tiltHead(20,2)
    pr2.rotateHead(-45, 2)
    joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
    values = [10,-125,70,90,-70,15, 0]
    pr2.moveArmJoint(joints,values, duration=2.0, wait=False)
    joints = ['r_shoulder_pan_joint','r_elbow_flex_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint','r_wrist_flex_joint','r_upper_arm_roll_joint', 'r_wrist_roll_joint']
    values = [-10,-125,70,-90,-70,-15, 0]
    pr2.moveArmJoint(joints,values, duration=2.0)
    pr2.setTorso(.05, 2.0)
    rospy.timer.sleep(15)
'''    
    'r_shoulder_pan_joint', 0
    'l_shoulder_pan_joint', 10
    'l_elbow_flex_joint', -125
    'l_shoulder_lift_joint', 70
    'l_forearm_roll_joint', 90
    'l_wrist_flex_joint', -70
    'l_upper_arm_roll_joint', 15
    'r_shoulder_pan_joint', -10
    'r_elbow_flex_joint', -125
    'r_forearm_roll_joint', -90
    'r_wrist_flex_joint', -70
    'r_upper_arm_roll_joint', -15
    pr2.setTorso(.05, 2.0)    
    joints = ['l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint']
    values = [-70, -90, -70]
    pr2.moveArmJoint(joints, values, duration=2.0, wait=False)
    joints = ['r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint']
    values = [70, 90, 70]
'''     
def personalSpace(newAngle, switch):

    #if switch = 1:        
        pr2.setTorso(.03 + pr2.getSensorReading("torso_lift_joint"), 1.0)
        joints = ['r_forearm_roll_joint', 'r_shoulder_lift_joint','r_wrist_roll_joint', 'r_wrist_flex_joint', 'r_upper_arm_roll_joint','r_shoulder_pan_joint'  ]
        value = [-90, 40, 90, 0, 0, 0]
        pr2.moveArmJoint(joints, value, duration=2.0, wait=False)
        joints = ['l_forearm_roll_joint', 'l_shoulder_lift_joint','l_wrist_roll_joint', 'l_wrist_flex_joint', 'l_upper_arm_roll_joint','l_shoulder_pan_joint'  ]
        value = [60, 70, 0, -80, 0, 15]
        pr2.moveArmJoint(joints, value, duration=2.0)
        pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=.5)
        pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=.5)
        pr2.moveArmJoint(['r_upper_arm_roll_joint'], [3], duration=.6)
        pr2.moveArmJoint(['r_upper_arm_roll_joint'], [-3], duration=.7)
        pr2.moveArmJoint(['r_upper_arm_roll_joint'], [0], duration=.8)
        rospy.timer.sleep(7)
        pr2.rotateHead(newAngle + 2, .5)
        rospy.timer.sleep(2)
        pr2.tiltHead(-5, .5)
        rospy.timer.sleep(6)
        pr2.rotateHead(newAngle, .5)
        rospy.timer.sleep(14)
        pr2.setTorso(-.01 + pr2.getSensorReading("torso_lift_joint"), 4.0)
        joints = ['l_shoulder_pan_joint','l_elbow_flex_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint','l_wrist_flex_joint','l_upper_arm_roll_joint', 'l_wrist_roll_joint']
        values = [10,-125,70,90,-70,15, 0]
        pr2.moveArmJoint(joints,values, duration=3.0, wait=False)
        joints = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint','l_upper_arm_roll_joint']
        values = [newAngle *.2 , 70, 0]
        pr2.moveArmJoint(joints,values, duration=3.0)    

    #else switch = 0:
        pass
   
def humanMoved(personDistance, personAngle):
    # do something
    
    if personDistance <= 4:
        pr2.tiltHead(-8,1)
        pr2.rotateHead(personAngle, 1.5)
        
    if personDistance <= 3:       
        personalSpace(personAngle, 1)
        
    if personDistance >= 5:
        personalSpace(personAngle, 0)
        pr2.setTorso(.02, 4)
        
    #if newDistance < 
def main():
    __init__()
    rospy.timer.sleep(10)
    humanMoved(4,30) # Human now at 2m and 30deg
    rospy.timer.sleep(4)
    humanMoved(3,40) # Human now at 2m and 30deg
    rospy.timer.sleep(15)
    humanMoved(6,40)
    
main()

   


