#!/usr/bin/env python
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

from tf.transformations import quaternion_from_euler

import numpy as np

class MoveBase(object):
    def __init__(self):
        self._target_pose = self
        self._robot_pose  = None
        self._mba_client  = None

        self.odom_topic = '/p3dx/odom'
        self.cmd_vel_topic = '/p3dx/cmd_vel'
        
        self._odom_sub    = None
        self._cmd_vel_pub = None

        rospy.sleep(.5)
        self.check()
    
    def check(self, timeout=1.0):
        self._mba_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self._mba_client.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr('Move base action server not found')
            rospy.signal_shutdown("server not available!")
            return False
        
        rospy.loginfo('Move base action server [OK]')

        try:
            rospy.wait_for_message(self.odom_topic, Odometry, timeout=timeout)
            rospy.loginfo('Odometry [OK]')
        except:
            rospy.logerr('Odometry not found')
            return False

        return True
    
    def odom_cb(self, msg):
        pass

    def set_target(self, x, y, theta):
        self._target_pose.x = x
        self._target_pose.y = y
        self._target_pose.theta = theta

    def get_target_pose(self):
        np.array([self._target_pose.x, self._target_pose.y, self._target_pose.theta])

    def get_robot_pose(self):
        pass

    def go(self):
        try:
            result = self._call_mba_client()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def rotate(self):
        pass

    def _call_mba_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self._target_pose.x
        goal.target_pose.pose.position.y = self._target_pose.y

        goal.target_pose.pose.orientation = self._euler_to_quat()

        self._mba_client.send_goal(goal)

        wait = self._mba_client.wait_for_result()

        if not wait:
            rospy.logerr("server not available!")
        else:
            return self._mba_client.get_result()

    def _euler_to_quat(self):
        q = quaternion_from_euler(0, 0, self._target_pose.theta/180.0*np.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

if __name__ == "__main__":
    rospy.init_node('move_base_client_test')
    mb = MoveBase()
    mb.set_target(0, 0, 0)
    mb.go()