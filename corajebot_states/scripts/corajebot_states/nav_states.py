#!/usr/bin/env python

import rospy
import smach
import smach_ros
from base import MoveBase


class GoToPoseState(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'], input_keys=['pose']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.base = MoveBase()

    def execute(self, ud):
        if not self.base.check():
            return 'failed'
        
        x, y, theta = ud.pose
        self.base.set_target(x, y, theta)
        self.base.go()
        return 'succeeded'

class Recover(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed', 'prempted']):
        smach.State.__init__(self, outcomes)
    
    def execute(self, ud):
        return 'succeeded'