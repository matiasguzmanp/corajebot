#!/usr/bin/env python

import rospy
import smach
import smach_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from corajebot_famous.findhidingplace import *
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Pose
from apriltag_ros.msg import AprilTagDetectionArray
from corajebot_states.base import MoveBase
import tf2_ros
import tf2_geometry_msgs

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
                data = map_msg.data
                origin = [map_msg.info.origin.position.x, map_msg.info.origin.position.y, map_msg.info.origin.position.z]
                resolution = map_msg.info.resolution
                width = map_msg.info.width
                height = map_msg.info.height
                self._goal_calculator.load_map(data,width,height,origin,resolution)
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

        self._tag_pose = None
        self._tag_detections = None
        self.tag_detection_topic = '/tag_detections'
        self.tag_sub = rospy.Subscriber(self.tag_detection_topic, AprilTagDetectionArray, self.getTagPose)

        self.base = MoveBase()

    def execute(self, ud):
        init_theta = self.base._robot_pose.theta
        while not rospy.is_shutdown():
            init_theta += 10
            if len(self._tag_detections) > 0:
                ud.papa_position = self._tag_pose
                return 'succeeded'
            
            else:
                print(self.base._robot_pose.theta)
                self.base.rotate(init_theta)
                #self.base.rotate(360)
            rospy.sleep(1) # 1 [s]

            if rospy.Time.now().to_sec() - self.time > self.timeout:
                return 'failed'
            self.time = rospy.Time.now().to_sec()

    def getTagPose(self, msg) :
        self._tag_detections = msg.detections
        try:
            self._tag_pose = self._tag_detections[0].pose.pose.pose
            pose = Pose()
            pose.position.x = self._tag_pose.position.z
            pose.position.y = -self._tag_pose.position.x
            pose.position.z = -self._tag_pose.position.y
            
            transformed_pose = self.transform_pose(pose, "camera_link", "map")
            self._tag_pose = transformed_pose
        except:
            pass
    def transform_pose(self, input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


class CalculateSafePosition(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'],
                       input_keys=['papa_position', 'map'], 
                       output_keys=['safe_pose'],
                       goal_calculator=None):

        smach.State.__init__(self, outcomes, input_keys=input_keys, output_keys=output_keys)
        self._goal_calculator = goal_calculator

    def execute(self, ud):
        try:
            paparazzi = ud.papa_position.position.z, ud.papa_position.position.x
            print("POSICION DE PAPA", ud.papa_position)
            msg = rospy.wait_for_message("/amcl_pose", Odometry)
            robot =  msg.pose.pose.position.x, msg.pose.pose.position.y 
            _, pos = self._goal_calculator.find_hiding_place(paparazzi, robot)
            ud.safe_pose = [pos[0], pos[1], 0]
            #ud.safe_pose = [1, 3, 0]
            return 'succeeded'
        except:
            rospy.loginfo('Could not find appropiate hidding place')
            return 'failed'
