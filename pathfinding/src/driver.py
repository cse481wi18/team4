
import actionlib
import rospy
import copy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import threading

TOLERANCE = 0.3

def printPose(stampedPose):
    pose = stampedPose.pose.pose
    print "Position: ", pose.position.x, pose.position.y, pose.position.z
    print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

muh_position = None


class Driver:
    def __init__(self):
        print "calling init", threading.current_thread().name
        # self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self._curr_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.set_curr_map_pose)
        self._goto_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._curr_goal_pose = None
        self._curr_map_pose = None

    # msg
    def set_curr_map_pose(self, curr_pose_msg):
        global muh_position
        print "setting set_curr_map_pose to ", curr_pose_msg
        print "curr thread set curr_map_pose: ", threading.current_thread().name
        muh_position = curr_pose_msg.pose.pose
        # print curr_pose_msg.pose.pose
        if curr_pose_msg is not None:
            self._curr_map_pose = curr_pose_msg.pose.pose # smthing of type Pose
        return
#         # TODO debug error message:
#         / brain
#         _invoke_callback:723: bad
#         callback: < bound
#         method
#         Driver.set_curr_map_pose
#         of < driver.Driver
#         object
#         at
#         0x7f4624291b90 >>
#         Traceback(most
#         recent
#         call
#         last):
#         File
#         "/opt/ros/indigo/lib/python2.7/dist-packages/rospy/topics.py", line
#         720, in _invoke_callback
#         cb(msg)
#
#     File
#     "/home/team4/catkin_ws/src/cse481wi18/pathfinding/src/driver.py", line
#     32, in set_curr_map_pose
#     if self._curr_goal_pose is not None and self._curr_map_pose and self.within_tolerance(self._curr_goal_pose,
#                                                                                           TOLERANCE):
#
#
# AttributeError: 'Driver'
# object
# has
# no
# attribute
# '_curr_goal_pose'

# if self._curr_goal_pose is not None and self._curr_map_pose and self.within_tolerance(self._curr_goal_pose, TOLERANCE):
        #     self.cancel_goals()
        #     self._curr_goal_pose = None

    def go_to(self, pose):
        global muh_position
        print "going to ", pose
        print "currently at (field var)", self._curr_map_pose
        print "currently at (global var)", muh_position
        print "curr thread set go_to: ", threading.current_thread().name

        if muh_position is None:
            rospy.loginfo("Help! I'm lost!")
            return
        self._curr_goal_pose = pose

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map" # base link?
        goal_pose.pose = copy.deepcopy(pose)
        quat_arr = tft.quaternion_from_euler(0, 0, math.atan2(pose.position.y - muh_position.position.y,
                                                              pose.position.x - muh_position.position.x))
        orientation = Quaternion(quat_arr[0], quat_arr[1], quat_arr[2], quat_arr[3])
        goal_pose.pose.orientation = orientation
        move_goal = MoveBaseGoal()
        move_goal.target_pose = goal_pose
        self._goto_client.send_goal_and_wait(move_goal) # blocks until move is successful
        self._curr_goal_pose = None

    def slowly_go_to(self, pose):
        pass

    def within_tolerance(self, target, tolerance):
        # print "currently at ", self._curr_map_pose
        if self._curr_map_pose is None and muh_position is None:
            print "[Driver:within_tolerane] curr_map_pose not set"
            return False
        dist = math.sqrt(math.pow((target.position.x - muh_position.position.x), 2) + math.pow((target.position.y - muh_position.position.y), 2))
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
