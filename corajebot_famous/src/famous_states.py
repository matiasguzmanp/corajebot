import rospy
import smach
import smach_ros
from nav_msgs.msg import OccupancyGrid
from corajebot_scripts.src.findhidingplace import find_hiding_place

class LoadMapState(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'], output_keys=['map'], timeout=2.0):
        smach.State.__init__(outcomes, output_keys)
        self.timeout = timeout

    def execute(self, ud):
        try:
            map_msg = rospy.wait_for_message("/map", timeout=self.timeout)
            ud.map = map_msg
            # aqui ver bien como inicializar la wea y donde dejar el mapa en vola es mejor incializar clase
            # o no se jsjjsa
            rospy.loginfo("Map loaded succesfully")
        except:
            rospy.logerr("Error, no map received")
            return 'failed'

        return 'succeeded'


class WaitForPaparazzi(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed', 'preempted'], output_keys=['papa_position'], timeout=1000):
        smach.State.__init__(outcomes, output_keys)
        self.time = rospy.Time.now().to_sec()
        self.timeout = timeout

    def execute(self, ud):
        while not rospy.is_shutdown():
            # try detecting april tag
            # if detection == bien:
            #   ud.papa_position = detected_position
            #   return 'succeded'
            # else:
            # rotar la base x graados o dar una vuelta completa lenta
            rospy.sleep(1) # 1 [s]

            if rospy.Time.now().to_sec() - self.time > self.timeout:
                return 'failed'
            self.time = rospy.Time.now().to_sec()

class CalculateSafePosition(smach.State):
    def __init__(self, outcomes=['succeeded', 'failed'], input_keys=['papa_position', 'map'], output_keys=['safe_pose']):
        smach.State.__init__(outcomes, input_keys, output_keys)
    
    def execute(self, ud):
        try:
            # ud.safe_pose = find_hiding_place(**args)
            return 'succeeded'
        except:
            rospy.loginfo('Could not find appropiate hidding place')
            return 'failed'

