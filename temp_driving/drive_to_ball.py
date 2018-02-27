#! /usr/bin/env python

import rospy
import copy
import pickle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header

current_position = None
FILE_NAME = "savedPoses.p"


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class Driver(object):
    def __init__(self):
        self._goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def createMarker(self, name, pose):
        new_marker = InteractiveMarker();

        new_marker.header.frame_id = "map"
        new_marker.name = name
        new_marker.description = name
        if pose is None:
            new_marker.pose.position.x = 0.0
            new_marker.pose.position.y = 0.0
            new_marker.pose.position.z = 0.1
            new_marker.pose.orientation.x = 0.0
            new_marker.pose.orientation.y = 0.0
            new_marker.pose.orientation.z = 0.0
            new_marker.pose.orientation.w = 1.0
        else:
            new_marker.pose = pose

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.40
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        pose_marker_control = InteractiveMarkerControl()
        pose_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        pose_marker_control.orientation.w = 1;
        pose_marker_control.orientation.x = 0;
        pose_marker_control.orientation.y = 1;
        pose_marker_control.orientation.z = 0;
        pose_marker_control.always_visible = True
        pose_marker_control.markers.append(arrow_marker)
        new_marker.controls.append(pose_marker_control)

        pose_rotation_control = InteractiveMarkerControl()
        pose_rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        pose_rotation_control.orientation.w = 1;
        pose_rotation_control.orientation.x = 0;
        pose_rotation_control.orientation.y = 1;
        pose_rotation_control.orientation.z = 0;
        pose_rotation_control.always_visible = True
        new_marker.controls.append(pose_rotation_control)

        self.marker_server.insert(new_marker, self.updatePosition)
        self.marker_server.applyChanges()

        # Copy? Will this update? How does the front end update position?
        self.PoseList[name] = copy.deepcopy(new_marker.pose)
        self.savePoses()


    def printPose(self, stampedPose):
        # current_pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=updateCurrentPose)

        pose = stampedPose.pose.pose

        print "Position: ", pose.position.x, pose.position.y, pose.position.z
        print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w


    def updateCurrentPose(self, msg):
        global current_position
        current_position = msg

    def go_to(self, pose_stamped):
        newGoal = PoseStamped()
        newGoal.header = copy.deepcopy(pose_stamped.header)
        newGoal.header.frame_id = "map"
        newGoal.pose = copy.deepcopy(pose_stamped.pose)

        self._goto_publisher.publish(newGoal)
