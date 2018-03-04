
import actionlib
import rospy
import copy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def printPose(stampedPose):
    pose = stampedPose.pose.pose

    print "Position: ", pose.position.x, pose.position.y, pose.position.z
    print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w


class Driver(object):
    def __init__(self):
        # self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.curr_goal_pose = None


    def go_to(self, pose_stamped):
        self.curr_goal_pose = pose_stamped
        print "going to ", pose_stamped

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose = copy.deepcopy(pose_stamped)
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal_pose
        self._goto_client.send_goal_and_wait(move_goal) #blocks until move is successful
        self.curr_goal_pose = None

    # TODO blocking
    def return_to_default_position(self, pose_stamped):
        pass
