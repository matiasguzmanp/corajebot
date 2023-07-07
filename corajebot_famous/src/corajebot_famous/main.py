#!/usr/bin/env python

import rospy
import smach
import smach_ros
 
from famous_states import LoadMapState, CalculateSafePosition, WaitForPaparazzi
from corajebot_states.nav_states import GoToPoseState, Recover



def getInstance():
    input_keys = []
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'prempted', 'timeout'], input_keys=input_keys)

    with sm:
        smach.StateMachine.add('INIT', LoadMapState(timeout=2.0),
            transitions = {
                'succeeded' : 'WAIT4PAPARAZZI',
                'failed'    : 'timeout' 
            })

        smach.StateMachine.add('WAIT4PAPARAZZI', WaitForPaparazzi(timeout=1000),
            transitions={
                'succeeded' : 'GETFREEPOSITION',
                'failed'    : 'timeout'
            })
        
        smach.StateMachine.add('GETFREEPOSITION', CalculateSafePosition(),
            transitions={
                'succeeded' : 'GOTOPOSE',
                'failed'    : 'failed'
            })
        
        smach.StateMachine.add('GOTOPOSE', GoToPoseState(),
            transitions={
                'succeeded' : 'WAIT4PAPARAZZI',
                'failed'    : 'RECOVERY'
            })
        
        smach.StateMachine.add('RECOVERY', Recover(),
            transitions={
                'succeeded'  : 'GOTOPOSE',
                'prempted'   : 'RECOVERY',
                'failed'     : 'failed'
            })
        


    return sm


if __name__ == "__main__":
    rospy.init_node('famous_node')
    print('aaaaaaaaaaaaaaaaaa')
    sm = getInstance()
    outcome = sm.execute()