#!/usr/bin/env python

import rospy
import smach
import smach_ros
from base import MoveBase


class GoToPoseState(smach.State):
    def __init__(self, outcomes=['succeded', 'failed', 'preemted'], input_keys=['pose']):
        smach.State.__init__(outcomes, input_keys)
        self.base = MoveBase()

    def execute(self, ud):
        if not self.base.check():
            return 'preempted'
        
        x, y, theta = ud.pose
        self.base.set_target(x, y, theta)
        self.base.go()
        return 'succeeded'

class Recover(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed']):
        smach.State.__init__(outcomes)
    
    def execute(self, ud):
        return 'succeeded'