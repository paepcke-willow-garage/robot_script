#!/usr/bin/env python

# This example script illustrates how robot scripts may 
# use the fake-human event simulator. A ball approaches,
# and retreats from the robot according to a specified
# schedule. 

# Before you start this script, quit Gazebo, and type this
# into a terminal:
#
#    roslaunch robot_script fakeHumanExampleNeededServices.launch

from robot_scripting import PR2RobotScript as pr2
from robot_scripting import aboutEq
from event_simulators.event_simulator import EventSimulator
from Queue import Empty
from collections import OrderedDict
from event_simulators.fakeHuman import FakeHuman 

human = FakeHuman()

# Create a schedule:
motionSchedule = OrderedDict();

# In the following motionSchedule keys (3,6, and 9) are seconds from program start.
# the 'target' numbers are meters of distance from the robot, and the 
# two speed components are meters/sec in the x and y direction, respectively.

motionSchedule[3]  = {'target': [2,0], 'speed': [0.5,0.0]}; 
motionSchedule[6]  = {'target': [3,0], 'speed': [0.5,0.0]}; 
motionSchedule[9]  = {'target': [4,0], 'speed': [0.5,0.0]};

# When the scheduled pose sequence is completed, it starts over.
# Start the sequence with a call to a FakeHuman's start() method, passing
# it a schedule. Stop the motions via a call to the stop() method:
    
human.start(motionSchedule);

# Obtain a queue from which human distances are obtained as
# they change:
eventQueue = human.getEventQueue();

while (True):
    # Wait for a new position report from the human:
    newHumanLocation = eventQueue.get();
    print('Human now at ' + str(newHumanLocation));

