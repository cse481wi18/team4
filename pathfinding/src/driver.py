
import actionlib
import rospy
import copy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math


def printPose(stampedPose):
    pose = stampedPose.pose.pose

    print "Position: ", pose.position.x, pose.position.y, pose.position.z
    print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w


class Driver(object):
    def __init__(self):
        # self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._curr_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.set_curr_map_pose)
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._curr_goal_pose = None
        self._curr_map_pose = None

    # msg
    def set_curr_map_pose(self, curr_pose):
        self._curr_map_pose = curr_pose.pose.pose.position

    def go_to(self, pose):
        self._curr_goal_pose = pose
        print "going to ", pose

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose = copy.deepcopy(pose)
        # quat_arr = tft.quaternion_from_euler(0, 0, math.atan2(pose.position.y - self._curr_map_pose.y, pose.position.x - self._curr_map_pose.x))
        # orientation = Quaternion(quat_arr[0], quat_arr[1], quat_arr[2], quat_arr[3])
        # goal_pose.pose.orientation = orientation #TODO figure out if we need - add to BallPositionsMsg
        goal_pose.pose.orientation = Quaternion()
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal_pose
        self._goto_client.send_goal_and_wait(move_goal) # blocks until move is successful
        self._curr_goal_pose = None

    def cancel_goals(self):
        self._goto_client.cancelAllGoals()

    # TODO blocking - milestone 2/3
    def return_to_default_position(self, pose_stamped):
        pass
