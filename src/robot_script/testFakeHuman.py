#!/usr/bin/env python

# This example script illustrates how robot scripts may 
# use the fake-human event simulator. A ball approaches,
# and retreats from the robot according to a specified
# schedule. 

# Before you start this script, quit Gazebo, and type this
# into a terminal:
#
#    roslaunch robot_script fakeHumanExampleNeededServices.launch

import signal;

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import RunMotion
from robot_scripting import aboutEq
from event_simulators.event_simulator import EventSimulator
from Queue import Empty
from collections import OrderedDict
from event_simulators.fakeHuman import FakeHuman 

LEFT  = pr2.LEFT
RIGHT = pr2.RIGHT
BOTH  = pr2.BOTH

human = FakeHuman()

# Create a schedule:
motionSchedule = OrderedDict();

# In the following motionSchedule keys (3,6, and 9) are seconds from program start.
# the 'target' numbers are meters of distance from the robot, and the 
# two speed components are meters/sec in the x and y direction, respectively.

motionSchedule[3]   = {'target': [2,0], 'speed': [0.05,0.0]}; 
motionSchedule[10]  = {'target': [3,0], 'speed': [0.05,0.0]}; 
motionSchedule[14]  = {'target': [4,0], 'speed': [0.05,0.0]};

# When the scheduled pose sequence is completed, it starts over.
# Start the sequence with a call to a FakeHuman's start() method, passing
# it a schedule. Stop the motions via a call to the stop() method:
    
human.start(motionSchedule);

def longMotionSequence():
    '''
    This function runs the robot through some motion forever.
    It doesn't need to. It could just be a long-running motion
    which does eventually stop.
    '''
    while True:
        pr2.openGripper(LEFT)
        pr2.panHead(-60)
        pr2.moveArmJoint('l_shoulder_pan_joint', 0, duration=2.0)            
        pr2.moveArmJoint('l_shoulder_pan_joint', 90, duration=2.0)
        pr2.panHead(0)
        pr2.closeGripper(LEFT)

# Obtain a queue from which human distances are obtained as
# they change:
eventQueue = human.getEventQueue();

myMotion = None

# Motions, like the function "longMotionSequence" above are 
# started using the following statement. Note that you 
# can currently only have one motion sequence running at
# the same time. You can stop a motion anytime by calling
# myMotion.stop(). For pausing a motion, see below:
myMotion = RunMotion(longMotionSequence)

while (True):
    # Wait for a new position report from the human:
    newHumanLocation = eventQueue.get();
    print('Human now at ' + str(newHumanLocation));
    
    # Depending on the human's location, pause or continue
    # the motion sequence:
    if newHumanLocation == 4.0 and myMotion is not None:
        myMotion.pause();
    if newHumanLocation < 3.0 and myMotion is not None:
        myMotion.resume();
