import rospy
from perception_msgs.msg import BallPositions
from geometry_msgs.msg import Pose
import fetch_api

NEGATIVE = ["fuck","fiddlesticks", "fudgesicle", "crap"]

class Speaker:
    def __init__(self):
        self._sound = fetch_api.RobotSound()
        self._curr_ind = 0

    # return None if no ball found
    # return some type of position (pose_stamped) (correspond to pick_up_ball arg)
    def say_negative(self):
        self._sound.say(NEGATIVE[self._curr_ind])
        self._curr_ind = (self._curr_ind + 1) % len(NEGATIVE)


