
import math
import rospy

class GripperJoints(object):
    def __init__(self):
        self._left_grip = 0
        self._right_grip = 0

    @staticmethod
    def names():
        return [
            'l_gripper_finger_joint', 'r_gripper_finger_joint'
        ]

    def values(self):
        return [
            self._left_grip, self._right_grip
        ]