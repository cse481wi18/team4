#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import fetch_api
import rospy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


def create_marker(posestamped):
    gripper_body = Marker()
    gripper_body.type = Marker.MESH_RESOURCE
    gripper_body.mesh_resource = GRIPPER_MESH
    left_finger = Marker()
    left_finger.type = Marker.MESH_RESOURCE
    left_finger.mesh_resource = L_FINGER_MESH
    right_finger = Marker()
    right_finger.type = Marker.MESH_RESOURCE
    right_finger.mesh_resource = R_FINGER_MESH


def make_6of_controls():
    # body = create_marker()


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "gripper_link"  # Could also be wrist_roll_link?
        gripper_im.name = "gripper_teleop_marker"
        gripper_im.description = "Gripper"
        gripper_im.pose.orientation.w = 1
        # create teal cube marker for the interactive marker
        fbox_marker = Marker()
        fbox_marker.type = Marker.CUBE
        fbox_marker.pose.orientation.w = 1
        fbox_marker.scale.x = 0.45
        fbox_marker.scale.y = 0.45
        fbox_marker.scale.z = 0.45
        fbox_marker.color.r = 0.5
        fbox_marker.color.g = 0.5
        fbox_marker.color.b = 0.0
        fbox_marker.color.a = 1.0
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
        """
        obj_im.header.frame_id = "base_link"
        obj_im.name = "auto_pick_teleop_marker"
        obj_im.description = "AutoPick"
        obj_im.pose.orientation.w = 1
        # create teal cube marker for the interactive marker
        fbox_marker = Marker()
        fbox_marker.type = Marker.CUBE
        fbox_marker.pose.orientation.w = 1
        fbox_marker.scale.x = 0.45
        fbox_marker.scale.y = 0.45
        fbox_marker.scale.z = 0.45
        fbox_marker.color.r = 0.5
        fbox_marker.color.g = 0.5
        fbox_marker.color.b = 0.0
        fbox_marker.color.a = 1.0
        """
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('nav_node')
    # wait_for_time()
    rospy.sleep(0.5)
    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()


if __name__ == '__main__':
    main()
