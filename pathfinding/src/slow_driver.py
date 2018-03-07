
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

class Driver:
    def __init__(self):
        self._base = fetch_api.Base()

    def go_forward(self, distance = DEFAULT_FORWARD_DISTANCE):
        self._base.go_forward(distance)


    # require: input in base_link (so robot's position is (0, 0)
    def turn_towards(self, target_pose_in_base_link):
        # treat robot vector as (0, 1)
        angle_in_rad = math.atan2(target_pose_in_base_link.position.y - 1, target_pose_in_base_link.position.x)
        calculated_angle_in_deg = math.degrees(angle_in_rad)
        self._base.turn(calculated_angle_in_deg)


    # TODO
    def get_position_offset_target(self, target):
        target.position.x = target.position.x - 0.5
        target.position.z = 0.0
        return target

