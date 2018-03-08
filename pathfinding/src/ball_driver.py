
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
DEFAULT_FORWARD_DISTANCE = 0.07
DEFAULT_REACHABLE_DISTANCE = 0.04

class Driver:
    def __init__(self):
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._base = fetch_api.Base()

    def go_forward(self, distance=DEFAULT_FORWARD_DISTANCE):
        print "going forward by: ", distance
        self._base.go_forward(distance)

    def go_to(self, target):
        self.turn_towards(target)
        distance = self.get_target_distance(target)
        self.go_forward(distance) # TODO y or x?

        # require: input in base_link (so robot's position is (0, 0)
    def turn_towards(self, target_pose_in_base_link):
        # treat robot vector as (0, 1)
        angle_in_rad = math.atan2(target_pose_in_base_link.position.y, target_pose_in_base_link.position.x)
        print "turning by: ", angle_in_rad
        # calculated_angle_in_deg = math.degrees(angle_in_rad)
        # Note: base.turn takes radians as an argument
        self._base.turn(angle_in_rad)

    # TODO - implement in arm controller
    def within_tolerance(self, target, tolerance):
        return False

    def get_target_distance(self, target):
        return target.position.y - DEFAULT_REACHABLE_DISTANCE

