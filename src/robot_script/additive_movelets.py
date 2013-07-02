#!/usr/bin/env python

from threading import Thread
import time


class Behavior(Thread):
    
    def __init__(self):
        super(Behavior,self).__init__()
        self.keepRunning = False
        self.behaviorSingleton = Behavior.BehaviorCombo(self)
    
    def __add__(self, other):
        return Behavior.BehaviorCombo(self,other)

    def __call__(self):
        self.start()
                
    def _stop(self):
        self.keepRunning = False
        
    def stop(self):
        self.behaviorSingleton.stop()

    class BehaviorCombo(object):
        
        def __init__(self, *behaviors):
            self.behaviors = behaviors
            
        def stop(self):
            for behavior in self.behaviors:
                behavior._stop()
            
        def __call__(self):
            for behavior in self.behaviors:
                behavior.start()


class Waver(Behavior):
    
    def run(self):
        self.keepRunning = True
        while self.keepRunning:
            print("waving")
            time.sleep(1)
    
class Nodder(Behavior):
    
    def run(self):
        self.keepRunning = True
        while self.keepRunning:
            print("    nodding")
            time.sleep(1)
        
        
if __name__ == '__main__':
    
    waver = Waver()
    waver()
    time.sleep(3)
    waver.stop()
    
    nodder = Nodder()
    nodder()
    time.sleep(3)
    nodder.stop()    
    
    combo = waver + nodder
    combo()
    time.sleep(5)
    combo.stop()
    
                    