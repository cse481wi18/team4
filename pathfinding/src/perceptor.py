import rospy
from perception_msgs.msg import BallPositions
from geometry_msgs.msg import Pose


class Perceptor:
    def __init__(self):
        self._get_ball_pub = rospy.Publisher('get_ball_locations', Pose, queue_size=5) # type of message does not matter

    # return None if no ball found
    # return some type of position (pose_stamped) (correspond to pick_up_ball arg)
    def get_closest_ball_location(self):
        print "[Perceptor: calling get_closest_ball_location]"
        # send request for message
        self._get_ball_pub.publish(Pose())
        # block until we get a response (balls or no balls)
        ball_positions_msg = rospy.wait_for_message("tennis_ball_position_topic", BallPositions)
        curr_poses = ball_positions_msg.positions
        # currently returns first ball in arr, TODO get closest ball
        if len(curr_poses) is not 0:
            return curr_poses[0]
        print "[Perceptor: no balls found]"
        return None


