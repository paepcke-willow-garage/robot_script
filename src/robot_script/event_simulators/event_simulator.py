#!/usr/bin/env python

import threading;
from Queue import Queue;
from Queue import Empty;
import time;
from collections import OrderedDict;
import signal

import rospy;

class EventSimulator(threading.Thread):
    '''
    Superclass for all robot_script event simulation applications. 
    Takes a schedule and a callback, and invokes the callback at
    times determined by the schedule. Each callback invokation is
    passed an argument that is specified by the schedule. Example
    schedule::
    
        	schedule = OrderedDict();
        	schedule[2.0] = 'This';
        	schedule[5.0] = 'is';
        	schedule[6.0] = 'a';
        	schedule[7.2] = 'test';
	       
    This schedule invokes the callback at 2.0 seconds from program start,
    and at 5, 6, and 7.2 seconds. The callback argument is the dict value
    corresponding to the time. See example in __main__.
    
    The simulator runs in a separate thread, which is started by calling
    start(). Make sure that your subclasses invoked this superclass' start()
    method.
    
    The callback function may return a value. This value is pushed into an
    eventQueue that may be access by other applications. Applications may
    obtain the eventQueue via getEventQueue(). See Python's built-in Queue
    class for how applications may feed from that queue.
    '''
    
    def start(self, schedule, callback, repeat=False):
        '''
        Starts the simulator. 
        @param schedule: plan with times and callback arguments. 
        @type schedule: collections.OrderedDict
        @param callback: callable to invoke at the times indicated in the schedule.
        @type callback: callable
        @param repeat: if True, schedule is excecuted over and over.
        @type repeat: boolean
        '''
        threading.Thread.__init__(self);
        if schedule is None or len(schedule) == 0:
            raise ValueError("Schedule for EventSimulator must have at least one entry.")
        self.schedule    = schedule;
        self.callback    = callback;
        self.repeat      = repeat;
        self.eventQueue  = Queue();
        self.keepRunning = True;
        signal.signal(signal.SIGINT, self.sigintHandler);    
        super(EventSimulator,self).start();
        
    def getEventQueue(self):
        '''
        Return this event simulator's event queue. This queue is fed with
        return values from the callbacks.
        '''
        return self.eventQueue;
        
    def run(self):
        '''
        Thread loop: works through the schedule, sleeping as needed.
        '''
        while (self.keepRunning and not rospy.is_shutdown()):
            # Start time:
            prevKeyframeTime = 0;
            for actionTime in self.schedule.keys():
                # Schedule is in absolute number of seconds since program
                # start. So compute distance between this and prev
                # schedule entry:
                sleepTime = actionTime - prevKeyframeTime; 
                time.sleep(sleepTime);
                if not self.keepRunning:
                    return;
                prevKeyframeTime = actionTime;
                result = self.callback(self.schedule[actionTime]);
                if result is not None:
                    self.eventQueue.put(result);
                
            if self.repeat:
                continue;
            else:
                return;

    def stop(self):
        self.keepRunning = False;
        
    def sigintHandler(signum, frame):
        self.stop();

if __name__ == '__main__':

    import sys

    schedule = OrderedDict();
    schedule[2.0] = 'This';
    schedule[5.0] = 'is';
    schedule[6.0] = 'a';
    schedule[7.2] = 'test';
    
    class MyEvents(EventSimulator):
        
        def start(self, schedule, repeat=False):
            self.startTime = time.time();
            super(MyEvents, self).start(schedule, self.printTime, repeat=repeat);
        
        def printTime(self, word):
            print str(time.time() - self.startTime) + ": " + word;
            return None;

        
    MyEvents().start(schedule);
    
    class MyComplexEvents(EventSimulator):
        def start(self, schedule, repeat=False):
            self.startTime = time.time();
            super(MyComplexEvents, self).start(schedule, self.queuePrintLine, repeat=repeat);
        
        def queuePrintLine(self, word):
            return str(time.time() - self.startTime) + ": " + word + " (via queue)";

    eventSimulator = MyComplexEvents();
    eventSimulator.start(schedule);
    eventQueue = eventSimulator.getEventQueue();
    while (True):
        try:
            event = eventQueue.get(block=True, timeout=4.0);
        except Empty:
            print("Queue empty for more than 4 seconds. Quitting.")
            sys.exit();
        print event;
        
    
    
    
    
    
    
    