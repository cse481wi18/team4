import rospy
import copy
from perception_msgs.msg import BallPositions
from geometry_msgs.msg import Pose
# TODO milestone 1 implement class


def temp_fn(msg):
    print "called tmp"

class Perceptor:
    # return None if no ball found
    # return some type of position (pose_stamped) (correspond to pick_up_ball arg)
    def get_closest_ball_location(self):
        curr_poses = copy.deepcopy(self._curr_seen_ball_poses)
        # currently returns first ball in arr, TODO get closest ball
        if len(curr_poses) is not 0:
            return curr_poses[0]
        # fn should be non-blocking
        return None

    def __init__(self):
        self._sub = rospy.Subscriber('tennis_ball_position_topic', BallPositions, self._set_curr_map_pose)
        self._curr_seen_ball_poses = [] # type pose/position


    def _set_curr_map_pose(self, obj_feat_msg):
        print "calling _set_curr_map_pose"
        self._curr_seen_ball_poses = []
        # todo milestone 1 stuff to process msgs
        for i in range(obj_feat_msg.num_balls_found):
            pose = Pose()
            pose.position.x = obj_feat_msg.positions[3*i]
            pose.position.y = obj_feat_msg.positions[3*i + 1]
            pose.position.z = obj_feat_msg.positions[3*i + 2]
            self._curr_seen_ball_poses.append(pose)
