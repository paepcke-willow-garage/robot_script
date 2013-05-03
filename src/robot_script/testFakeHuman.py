#!/usr/bin/env python

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
motionSchedule = OrderedDict();
motionSchedule[2]  = None;
motionSchedule[5]  = {'target': [2,-1], 'speed': [0.1,0.03]}; 
motionSchedule[10] = None;
motionSchedule[12] = {'target': [4,-2], 'speed': [.1,.055]}; 
motionSchedule[16] = None;
motionSchedule[20] = {'target': [0.6,.5], 'speed': [.08,.1]};
motionSchedule[27] = None;    
    
human.start(motionSchedule);

eventQueue = human.getEventQueue();
while (True):
    newHumanLocation = eventQueue.get();
    print('Human now at ' + str(newHumanLocation));

