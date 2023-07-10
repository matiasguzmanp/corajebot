#!/usr/bin/env python

import rospy
import smach
import smach_ros
 
from famous_states import LoadMapAndSensorsState, CalculateSafePosition, WaitForPaparazzi
from corajebot_states.nav_states import GoToPoseState, Recover
from corajebot_famous.findhidingplace import FindPlaceToHide


def getInstance():
    calculator = FindPlaceToHide()
    input_keys = []

    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'prempted', 'timeout'], input_keys=input_keys)

    with sm:
        smach.StateMachine.add('INIT', LoadMapAndSensorsState(timeout=2.0, goal_calculator=calculator),
            transitions = {
                'succeeded' : 'WAIT4PAPARAZZI',
                'failed'    : 'timeout' 
            })

        smach.StateMachine.add('WAIT4PAPARAZZI', WaitForPaparazzi(timeout=1000),
            transitions={
                'succeeded' : 'GETFREEPOSITION',
                'failed'    : 'timeout'
            })
        
        smach.StateMachine.add('GETFREEPOSITION', CalculateSafePosition(goal_calculator=calculator),
            transitions={
                'succeeded' : 'GOTOPOSE',
                'failed'    : 'failed'
            })
        
        smach.StateMachine.add('GOTOPOSE', GoToPoseState(),
            transitions={
                'succeeded' : 'WAIT4PAPARAZZI',
                'failed'    : 'RECOVERY'
            },
            remapping={'pose' : 'safe_pose'})
        
        smach.StateMachine.add('RECOVERY', Recover(),
            transitions={
                'succeeded'  : 'GOTOPOSE',
                'prempted'   : 'RECOVERY',
                'failed'     : 'failed'
            })
        
    return sm


if __name__ == "__main__":
    rospy.init_node('famous_node')
    sm = getInstance()
    outcome = sm.execute()