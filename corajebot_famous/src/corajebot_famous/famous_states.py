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
                width, height = map_msg.info.width, map_msg.info.height

                self._goal_calculator.load_map(data, width, height, origin, resolution, is_lab=True)
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
        
        self.base = MoveBase()

    def execute(self, ud):
        ud.papa_position = None
        init_theta = self.base._robot_pose.theta
        while not rospy.is_shutdown():
            init_theta += 30
            msg = rospy.wait_for_message(self.tag_detection_topic, AprilTagDetectionArray)
            self._tag_pose = self.get_tag_pose(msg)
            if self._tag_pose is not None:
                print("Tag position: ", self._tag_pose)
                ud.papa_position = self._tag_pose
                self._tag_pose = None
                self._tag_detections = None

                return 'succeeded'
            
            else:
                # print(self.base._robot_pose.theta)
                self.base.rotate(init_theta)

            rospy.sleep(.5) # [s]

            if rospy.Time.now().to_sec() - self.time > self.timeout:
                return 'failed'
            self.time = rospy.Time.now().to_sec()

    def get_tag_pose(self, msg) :
        try:
            if len(msg.detections) != 0:
                tag_pose = msg.detections[0].pose.pose.pose
                pose = Pose()
                pose.position.y = -tag_pose.position.x
                pose.position.x =  tag_pose.position.z
                pose.position.z =  tag_pose.position.y

                return self.transform_pose(pose, "camera_link", "map")
            
            return None
        except:
            rospy.logwarn('Could not get transformed tag pose')
            return None

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
            paparazzi = ud.papa_position.position.x, ud.papa_position.position.y
            # print("POSICION DE PAPA", ud.papa_position)
            msg = rospy.wait_for_message("/amcl_pose", Odometry)
            robot =  msg.pose.pose.position.x, msg.pose.pose.position.y 
            pxs, pos = self._goal_calculator.find_hiding_place(paparazzi, robot)
            plt.imshow(self._goal_calculator.original_map, cmap='gray',origin='lower')
            plt.scatter(pxs[0],pxs[1], marker="D", label="Goal")
            plt.scatter(self._goal_calculator.paparazzi_in_pixels[0],self._goal_calculator.paparazzi_in_pixels[1], marker='x',label="Paparazzi")
            plt.scatter(self._goal_calculator.robot_in_pixels[0],self._goal_calculator.robot_in_pixels[1], marker='^', label="Robot")
            plt.legend()
            plt.show()

            plt.imshow(self._goal_calculator.distances_from_point, cmap='gray',origin='lower')
            plt.show()

            ud.safe_pose = [pos[0], pos[1], 0]
            # ud.safe_pose = [1, 0, 0]
            return 'succeeded'
        except:
            rospy.loginfo('Could not find appropiate hidding place')
            return 'failed'
