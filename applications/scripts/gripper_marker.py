#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import fetch_api
import rospy
import copy
from geometry_msgs.msg import PoseStamped

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'


def create_markers(pose_stamped):
    # teal gripper marker
    gripper_body = Marker()
    gripper_body.type = Marker.MESH_RESOURCE
    gripper_body.mesh_resource = GRIPPER_MESH
    gripper_body.pose.orientation.w = 1
    gripper_body.pose.position.x = .1
    gripper_body.scale.x = 0.45
    gripper_body.scale.y = 0.45
    gripper_body.scale.z = 0.45
    gripper_body.color.r = 1.0
    gripper_body.color.a = 1.0

    # left/right fingers - other colors
    left_finger = Marker()
    left_finger.type = Marker.MESH_RESOURCE
    left_finger.mesh_resource = L_FINGER_MESH
    left_finger.pose.orientation.w = 1
    left_finger.pose.position.x = .1
    left_finger.scale.x = 0.45
    left_finger.scale.y = 0.45
    left_finger.scale.z = 0.45
    left_finger.color.r = 0.5
    left_finger.color.a = 1.0

    right_finger = Marker()
    right_finger.type = Marker.MESH_RESOURCE
    right_finger.mesh_resource = L_FINGER_MESH
    right_finger.pose.orientation.w = 1
    right_finger.pose.position.x = .1
    right_finger.scale.x = 0.45
    right_finger.scale.y = 0.45
    right_finger.scale.z = 0.45
    right_finger.color.r = 1.0
    right_finger.color.a = 0.5


def make_6of_controls(): # TODO args?
    # body = create_marker()

MENU_GRIP_OPEN = 1
MENU_GRIP_CLOSE = 2
MENU_GRIP_MOVE = 3

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self.pose_ok = False
        self.pose = None

    #TODO difference between start and init?
    def start(self):
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "gripper_link"  # Could also be wrist_roll_link?  #TODO wait should this be baselink?
        gripper_im.name = "gripper_teleop_marker"
        gripper_im.description = "Gripper"
        create_markers()
        make_6of_controls()
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        gripper_marker = copy.deepcopy(self._im_server.get('gripper_teleop_marker'))
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == MENU_GRIP_OPEN:
                self._gripper.open()
            elif feedback.menu_entry_id == MENU_GRIP_CLOSE:
                self._gripper.close()
            elif feedback.menu_entry_id == MENU_GRIP_MOVE:
                kwargs = {
                    'allowed_planning_time': 10,
                    'execution_timeout': 10,
                    'num_planning_attempts': 5,
                    'replan': True,
                }
                if self.pose_ok:
                    self._arm.move_to_pose(self.pose, **kwargs)
                else:
                    rospy.logerr("Error - couldn't move to pose")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose_stamped = PoseStamped()
            pose_stamped.pose = feedback.pose
            pose_stamped.header = feedback.header
            result = self._arm.compute_ik(pose_stamped)
            self.pose_ok = result
            self.pose = pose_stamped
            self._im_server.insert(gripper_marker)
            self._im_server.applyChanges()


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        obj_im = InteractiveMarker()
        """
        obj_im.header.frame_id = "base_link" # wait should this be baselink?
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
    rospy.init_node('gripper_marker_node') # TODO I don't remember if this name matters
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
