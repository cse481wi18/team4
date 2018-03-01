
import rospy
import copy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header

current_position = None

# TODO milestone 1 implement class


class Driver(object):
    def __init__(self):
        self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


    def printPose(self, stampedPose):
        pose = stampedPose.pose.pose

        print "Position: ", pose.position.x, pose.position.y, pose.position.z
        print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w


    def updateCurrentPose(self, msg):
        global current_position
        current_position = msg

    def go_to(self, pose_stamped):
        # TODO blocking
        newGoal = PoseStamped()
        newGoal.header = copy.deepcopy(pose_stamped.header)
        newGoal.header.frame_id = "map"
        newGoal.pose = copy.deepcopy(pose_stamped.pose)

        self._goto_publisher.publish(newGoal)
        # TODO wait and return

    # TODO blocking
    def return_to_default_position(self, pose_stamped):
        pass