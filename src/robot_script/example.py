#!/usr/bin/env python

'''
B{List of available commands:}

Gripper and Head:
    - pr2.openGripper(side, wait=True)    # side is LEFT or RIGHT
    - pr2.closeGripper(side, wait=True)   # side is LEFT or RIGHT
    - pr2.tiltHead(-30, wait=True)        # can add something like 'duration=2.0' for motion duration 
    - pr2.rotateHead(30, wait=True)       # can add something like 'duration=2.0' for motion duration 
    - pr2.lookAt(-100, 30, wait=False)    # takes both pan and tilt angle for smooth motion 
 
Moving the arm joints one at a time, or together:
    - pr2.moveArmJoint(jointName, newAngle, duration=2.0, wait=False)
    - pr2.moveArmJoint([jointName1, jointName2, ...], [newAngle1, newAngle2, ...], duration=2.0, wait=True)

Moving the whole robot (the base):
    - pr2.moveBase(place=(x,y,z), rotation=deg, duration=2, wait=True)
 
Reading joint values:
    - pr2.getSensorReading(sensorName)

Checking approximate joint value (when setting a joint to 0, it's often something like 0.0000134; aboutEq() does an approximate compare)
    - aboutEq(jointName, value)

Raising/lowering the torso:
    - pr2.setTorso(.05, 2.0, wait=True)

Temporarily pausing execution (but already initiated robot motions continue): 
    - rospy.timer.sleep(seconds)

More examples:
	- pr2.tiltHead(15 + pr2.getSensorReading("head_tilt_joint"), 1)
	
	- joints = ['l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint']
	values = [-70, -90, -70]
	pr2.moveArmJoint(joints, values, duration=2.0, wait=False)

ADVANCED USE: Simulated events impacting your script:

    - A simulator, or sensor can provide callbacks to your 
    application at regular intervals, or on a schedule you
    determine::

        from event_simulator import EventSimulator
    
        # Callback invoked at specified times. Keep this
        # callback short. It runs in a different thread,
        # which should be free to generate further events.
        # For more sophisticated use, see next section
        # 'Even more advanced use.' 
        
        def printWord(word):
            print(word)
            
        schedule = OrderedDict()
        schedule[2.0] = 'This'
        schedule[5.0] = 'is'
        schedule[6.0] = 'a'
        schedule[7.2] = 'test'
    
        EventSimulator().start(schedule, printWord)

EVEN MORE ADVANCED USE:      

    - You can queue up results that are computed or sensed by
    the callbacks:
    The simulator callbacks may return values to the simulator.
    Each simulator is equipped with a queue into which it will
    place those returned values. Your application thread can
    pick them up from that queue.
    
    The following code creates a callback that upper-cases
    every word it receives. These results are queued for the
    application to pick from a queue::
    
        # Callback function that returns a value:
        
        def timeToDoSomething(word):
            return word.upper()
    
        mySimulator = EventSimulator() 
        eventSimulator.start(schedule, timeToDoSomething);
        eventQueue = eventSimulator.getEventQueue()
        while (True):
            try:
                event = eventQueue.get(block=True, timeout=4.0);
            except Empty:
                print("Queue empty for more than 4 seconds. Quitting.")
                sys.exit();
            print event;

     - See also file testFakeHuman.py for a particular event simulator.


The many 'if' statements below just ensure that the robot will do something
different each time you run this file.

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

if aboutEq('head_tilt_joint', 30):
    pr2.lookAt(-100, -20 , 5, wait=False)
else:
    pr2.lookAt(+100, +30 , 5, wait=False)

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


# Advanced: event simulators that generate events via callbacks:

from event_simulators.event_simulator import EventSimulator
from Queue import Empty
from collections import OrderedDict
import sys

def printWord(word):
    print(word)
    
# Display 'This is a test,' with words printed at 2,5,6, and 7.2 seconds:
schedule = OrderedDict()
schedule[2.0] = 'This'
schedule[5.0] = 'is'
schedule[6.0] = 'a'
schedule[7.2] = 'test'

EventSimulator().start(schedule, printWord)


# Even more advanced: callback 

# Note: this word upper-casing example will run simultaneously
#       with the example above. While the example above will
#       print the words within the callback function, the following
#       example has the callback function do the work, but this application
#       picks the result up from a thread-safe queue

def timeToDoSomething(word):
    return word.upper()
    
mySimulator = EventSimulator() 
mySimulator.start(schedule, timeToDoSomething);
eventQueue = mySimulator.getEventQueue()
while (True):
    try:
        event = eventQueue.get(block=True, timeout=4.0);
    except Empty:
        print("Queue empty for more than 4 seconds. Quitting.")
        sys.exit();
    print("From the event queue: " + event);
