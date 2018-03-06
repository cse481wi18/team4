import rospy
import copy
from perception_msgs.msg import BallPositions
from geometry_msgs.msg import Pose
# TODO milestone 1 implement class


class Perceptor:
    # return None if no ball found
    # return some type of position (pose_stamped) (correspond to pick_up_ball arg)
    def get_closest_ball_location(self):
        print "calling get_closest_ball_location"
        curr_poses = copy.deepcopy(self._curr_seen_ball_poses)
        # currently returns first ball in arr, TODO get closest ball
        if len(curr_poses) is not 0:
            return curr_poses[0]
        # fn should be non-blocking
        print "[Perceptor: no balls found]"
        return None

    def __init__(self):
        self._sub = rospy.Subscriber('tennis_ball_position_topic', BallPositions, self._set_curr_map_pose)
        self._curr_seen_ball_poses = [] # type Pose


    def _set_curr_map_pose(self, obj_feat_msg):
        print "calling _set_curr_map_pose"
        print "In perceptor, seeing: "
        for i in obj_feat_msg.positions:
            print i
        self._curr_seen_ball_poses = obj_feat_msg.positions