
import actionlib
import rospy
import copy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import fetch_api

TOLERANCE = 0.3
DEFAULT_FORWARD_DISTANCE = 0.05
DEFAULT_REACHABLE_DISTANCE = 0.6

class Driver:
    def __init__(self):
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._base = fetch_api.Base()

    def go_forward(self, distance=DEFAULT_FORWARD_DISTANCE):
        print "[ball_driver: going forward by: ", distance, "]"
        self._base.go_forward(distance)

    def go_to(self, target):
        self.turn_towards(target)
        distance = self.get_target_distance(target)
        self.go_forward(distance) # TODO y or x?

        # require: input in base_link (so robot's position is (0, 0)
    def turn_towards(self, target_pose_in_base_link):
        # treat robot vector as (0, 1)
        angle_in_rad = math.atan2(target_pose_in_base_link.position.y, target_pose_in_base_link.position.x)
        # base.turn(value * math.pi / 180)
        calculated_angle_in_deg = math.degrees(angle_in_rad)
        print "[ball_driver: turning by degrees: ", calculated_angle_in_deg, "]"
        print "[ball_driver: turning by radians: ", angle_in_rad, "]"
        # Note: base.turn takes radians as an argument
        self._base.turn(angle_in_rad)

    # TODO - implement in arm controller
    def within_tolerance(self, target, tolerance):
        return False

    # SHOULD DEFINITELY BE X
    def get_target_distance(self, target):
        distance = target.position.x - DEFAULT_REACHABLE_DISTANCE
        print "[ball_driver: caculated distance: ", distance, "]"
        if distance < 0:
            distance = 0
        return distance

