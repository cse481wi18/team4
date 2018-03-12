
import actionlib
import rospy
import copy
import pickle
import tf
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import threading
import numpy as np

TOLERANCE = 0.3
FILE_NAME = "tennis_ball_robot_positions.p"

def printPose(stampedPose):
    pose = stampedPose.pose.pose
    print "Position: ", pose.position.x, pose.position.y, pose.position.z
    print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

muh_position = None


class Driver:
    def __init__(self):
        # self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._curr_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.set_curr_map_pose)
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._curr_goal_pose = None
        self._curr_map_pose = None
        self._listener = tf.TransformListener()
        rospy.on_shutdown(self._on_shutdown)

    # msg
    def set_curr_map_pose(self, curr_pose_msg):
        global muh_position
        # print "curr thread set curr_map_pose: ", threading.current_thread().name
        muh_position = curr_pose_msg.pose.pose
        # print curr_pose_msg.pose.pose
        if curr_pose_msg is not None:
            self._curr_map_pose = curr_pose_msg.pose.pose # smthing of type Pose
        return

    # No tolerance adjustment
    def go_to(self, pose):
        print "canceling goals..."
        self._cancel_goals()
        rospy.sleep(3)
        print "[driver/go_to]Going to position: "
        print pose
        print "[driver/go_to]I think I'm at: "
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose = pose
        # if muh_position is None:
        #     rospy.loginfo("[Driver] Help! I'm lost! muh_position is not set!")
        #     return
        self._curr_goal_pose = pose
        move_goal = MoveBaseGoal()
        move_goal.target_pose = ps
        self._goto_client.send_goal_and_wait(move_goal) # blocks until move is successful
        print "[driver] done moving"
        self._curr_goal_pose = None


    def _cancel_goals(self):
        self._goto_client.cancel_all_goals()

    def _on_shutdown(self):
        self._cancel_goals()


