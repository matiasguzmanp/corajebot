#!/usr/bin/env python

import rospy
import smach
import smach_ros
from nav_msgs.msg import OccupancyGrid
from corajebot_famous.findhidingplace import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
from base import MoveBase

class LoadMapAndSensorsState(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'], 
                       output_keys=['map'], 
                       goal_calculator=None,
                       timeout=2.0):

        smach.State.__init__(self, outcomes, output_keys=output_keys)
        self.timeout = timeout
        self._goal_calculator = goal_calculator

    def execute(self, ud):
        try:
            if self._goal_calculator is not None:
                map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=self.timeout)
                # laser = rospy.wait_for_message("/p3dx/laser/scan")
                ud.map = map_msg

                #self._goal_calculator.load_map(map_msg)
                rospy.loginfo("Map loaded succesfully")
            else:
                rospy.logerr('No goal calculator received')
        except:
            rospy.logerr("No map received")
            return 'failed'

        return 'succeeded'


class WaitForPaparazzi(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed', 'prempted'], output_keys=['papa_position'], timeout=1000):
        smach.State.__init__(self, outcomes, output_keys=output_keys)
        self.time = rospy.Time.now().to_sec()
        self.timeout = timeout

        self._tag_pose = self
        self.tag_detection_topic = '/tag_detections'
        self.tag_sub = rospy.Subscriber(self.tag_detection_topic, AprilTagDetectionArray, self.getTagPose)

        self.base = MoveBase()

    def execute(self, ud):
        while not rospy.is_shutdown():

            if self._tag_pose != []:
                ud.papa_position = self._tag_pose
                return 'succeeded'
            
            else:
                self.base.rotate(5)
                #self.base.rotate(360)
            rospy.sleep(1) # 1 [s]

            if rospy.Time.now().to_sec() - self.time > self.timeout:
                return 'failed'
            self.time = rospy.Time.now().to_sec()

    def getTagPose(self, msg) :
        try:
            self._tag_pose = msg[0].PoseWithCovarianceStamped
            self._tag_pose.x = msg.pose.pose.position.x
            self._tag_pose.y = msg.pose.pose.position.y
            self._tag_pose.theta = self._yaw_from_quat(msg.pose.pose.orientation)
        except:
            pass


class CalculateSafePosition(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'],
                       input_keys=['papa_position', 'map'], 
                       output_keys=['safe_pose'],
                       goal_calculator=None):

        smach.State.__init__(self, outcomes, input_keys=input_keys, output_keys=output_keys)
        self._goal_calculator = goal_calculator

    def execute(self, ud):
        try:
            # _, pos = self._goal_calculator.find_hiding_place(**args)
            #ud.safe_pose = [pos[0], pos[1], 0]
            ud.safe_pose = [1, 3, 0]
            return 'succeeded'
        except:
            rospy.loginfo('Could not find appropiate hidding place')
            return 'failed'
