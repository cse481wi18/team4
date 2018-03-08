
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
FILE_NAME = "savedPoses.p"

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

    # msg
    def set_curr_map_pose(self, curr_pose_msg):
        global muh_position
        # print "setting set_curr_map_pose to ", curr_pose_msg
        # print "curr thread set curr_map_pose: ", threading.current_thread().name
        muh_position = curr_pose_msg.pose.pose
        # print curr_pose_msg.pose.pose
        if curr_pose_msg is not None:
            self._curr_map_pose = curr_pose_msg.pose.pose # smthing of type Pose
        return

    def go_to(self, pose):
        # self._goto_client.cancelAllGoals()
        global muh_position
        # convert inpot pose to map
        # try:
        self._listener.waitForTransform('base_link', 'map', rospy.Time(0), rospy.Duration(10.0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print "[Driver/go_to]Error transforming input pose!"
        #     return

        # construct poseStamped to transformPose
        to_transform_pose_stmped = PoseStamped()
        to_transform_pose_stmped.header.frame_id = 'base_link' # todo check
        to_transform_pose_stmped.pose = copy.deepcopy(pose)
        transformed_pose_stmped = self._listener.transformPose('map', to_transform_pose_stmped)

        print "going to transformed", transformed_pose_stmped
        # print "currently at (field var)", self._curr_map_pose
        # print "currently at (global var)", muh_position
        # print "curr thread calling go_to: ", threading.current_thread().name

        if muh_position is None:
            rospy.loginfo("Help! I'm lost!")
            return
        self._curr_goal_pose = transformed_pose_stmped

        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = "map" # TODO base link?
        # goal_pose.pose = copy.deepcopy(pose)
        quat_arr = tft.quaternion_from_euler(0, 0, math.atan2(transformed_pose_stmped.pose.position.y - muh_position.position.y,
                                                              transformed_pose_stmped.pose.position.x - muh_position.position.x))
        orientation = Quaternion(quat_arr[0], quat_arr[1], quat_arr[2], quat_arr[3])
        transformed_pose_stmped.pose.orientation = orientation
        move_goal = MoveBaseGoal()
        move_goal.target_pose = transformed_pose_stmped
        self._goto_client.send_goal_and_wait(move_goal) # blocks until move is successful
        self._curr_goal_pose = None


    def within_tolerance(self, target, tolerance):
        # print "currently at ", self._curr_map_pose
        if self._curr_map_pose is None and muh_position is None:
            print "[Driver:within_tolerane] curr_map_pose not set"
            return False

        tar = np.array([target.position.x, target.position.y])
        pos = np.array([muh_position.position.x, muh_position.position.y])
        dist = np.linalg.norm(tar - pos)
        # dist = math.sqrt(math.pow((target.position.x - muh_position.position.x), 2) + math.pow((target.position.y - muh_position.position.y), 2))
        print "[Driver.within_tolerance] calculated distance: ", dist
        return dist <= tolerance


    # def go_to(self, pose):
    #     self._curr_goal_pose = pose
    #     print "going to ", pose
    #     print "currently at ", self._curr_map_pose
    #
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = "base_link" # base link?
    #     goal_pose.pose = copy.deepcopy(pose)

    #
    #     # move_goal = MoveBaseGoal()
    #     # move_goal.target_pose = goal_pose
    #     print "calling goto"
    #     self._goto_publisher.publish(goal_pose) # blocks until move is successful
    #     rospy.sleep(65) # TODO CHANGE (lol)
    #     print "exiting goto"
    #     self._curr_goal_pose = None



    def cancel_goals(self):
        self._goto_client.cancelAllGoals()

    # TODO
    def get_position_offset_target(self, target):
        target.position.x = target.position.x - 0.5
        target.position.z = 0.0
        return target

    # TODO blocking - milestone 2/3
    def return_to_default_position(self, pose_stamped):
        pass

