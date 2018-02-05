#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from std_msgs.msg import ColorRGBA
import fetch_api
import rospy
import copy
from geometry_msgs.msg import PoseStamped

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

MENU_GRIP_OPEN = 1
MENU_GRIP_CLOSE = 2
MENU_GRIP_MOVE = 3

# TODO need offset: rosrun tf tf_echo wrist_roll_link gripper_link

"""
Notes from Justin:
only track position of the "interactive marker," the meshes are defined relative to the interactive marker -
don't set the poses of the meshes
Don't set the frame-ids of each mesh

For the autopick, have 3 gripper poses in frame of object, to convert to base frame:
multiply transform of base to object by transform of object to gripper to get transform from base to gripper
(bTo * oTg = bTg)
"""


def create_markers(gripper_interactive_marker):
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = True  # handle drag events

    # teal gripper marker
    gripper_body = Marker()
    gripper_body.type = Marker.MESH_RESOURCE
    gripper_body.mesh_resource = GRIPPER_MESH
    # gripper_body.pose.orientation.w = 1
    # gripper_body.pose.position.x = .1
    # gripper_body.scale.x = 0.45
    # gripper_body.scale.y = 0.45
    # gripper_body.scale.z = 0.45
    gripper_body.color = ColorRGBA(0, 0, 1, 1)
    control.markers.append(gripper_body)

    # left/right fingers - other colors
    left_finger = Marker()
    left_finger.type = Marker.MESH_RESOURCE
    left_finger.mesh_resource = L_FINGER_MESH
    # left_finger.pose.orientation.w = 1
    left_finger.pose.position.y = -.16
    # left_finger.scale.x = 0.45
    # left_finger.scale.y = 0.45
    # left_finger.scale.z = 0.45
    left_finger.color = ColorRGBA(0, 0, 1, 1)
    control.markers.append(left_finger)

    right_finger = Marker()
    right_finger.type = Marker.MESH_RESOURCE
    right_finger.mesh_resource = L_FINGER_MESH
    # right_finger.pose.orientation.w = 1
    right_finger.pose.position.y = -.06
    # right_finger.scale.x = 0.45
    # right_finger.scale.y = 0.45
    # right_finger.scale.z = 0.45
    right_finger.color = ColorRGBA(0, 0, 1, 1)
    control.markers.append(right_finger)

    gripper_interactive_marker.controls.append(control)  # dont remember if python is pass by value/ref


# Returns a list of InteractiveMarkerControls
def make_6of_controls():
    controls = []

    # TODO orientations for markers (different for move/rotate?)
    control_x = InteractiveMarkerControl()
    control_x.name = 'move_x'
    control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_x.always_visible = True
    controls.append(control_x)
    rot_control_x = copy.deepcopy(control_x)
    rot_control_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_control_x.name = 'rotate_x'
    controls.append(rot_control_x)

    control_y = InteractiveMarkerControl()
    control_y.name = 'move_y'
    control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_y.always_visible = True
    control_y.orientation.w = 1
    control_y.orientation.x = 0
    control_y.orientation.y = 1
    control_y.orientation.z = 0

    controls.append(control_y)
    rot_control_y = copy.deepcopy(control_y)
    rot_control_y.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_control_y.name = 'rotate_y'
    rot_control_y.orientation.w = 1
    rot_control_y.orientation.x = 0
    rot_control_y.orientation.y = 1
    rot_control_y.orientation.z = 0

    controls.append(rot_control_y)

    control_z = InteractiveMarkerControl()
    control_z.name = 'move_z'
    control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_z.always_visible = True
    control_z.orientation.w = 1
    control_z.orientation.x = 0
    control_z.orientation.y = 0
    control_z.orientation.z = 1

    controls.append(control_z)
    rot_control_z = copy.deepcopy(control_z)
    rot_control_z.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_control_z.name = 'rotate_z'
    rot_control_z.orientation.w = 1
    rot_control_z.orientation.x = 0
    rot_control_z.orientation.y = 0
    rot_control_z.orientation.z = 1
    controls.append(rot_control_z)
    return controls


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self.pose_ok = False
        self.pose = None
        self.gripper_im = None

    # TODO difference between start and init?
    def start(self):
        self.gripper_im = InteractiveMarker()
        self.gripper_im.header.frame_id = "gripper_link"
        self.gripper_im.name = "gripper_teleop_marker"
        self.gripper_im.description = "Gripper"
        create_markers(self.gripper_im)
        controls = make_6of_controls()
        self.gripper_im.controls.extend(controls)

        open_entry = MenuEntry()
        open_entry.id = MENU_GRIP_OPEN
        open_entry.title = "Open"
        self.gripper_im.menu_entries.append(open_entry)
        close_entry = MenuEntry()
        close_entry.id = MENU_GRIP_CLOSE
        close_entry.title = "Close"
        self.gripper_im.menu_entries.append(close_entry)
        move_entry = MenuEntry()
        move_entry.id = MENU_GRIP_MOVE
        move_entry.title = "Move"
        self.gripper_im.menu_entries.append(move_entry)

        self._im_server.insert(self.gripper_im, feedback_cb=self.handle_feedback)
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
            pose_stamped.pose = copy.deepcopy(feedback.pose)
            pose_stamped.header = copy.deepcopy(feedback.header)
            result = self._arm.compute_ik(pose_stamped)
            self.pose_ok = result
            for mesh in gripper_marker.controls[0].markers:
                if result:
                    mesh.color = ColorRGBA(0, 1, 0, 1)
                else:
                    mesh.color = ColorRGBA(1, 0, 0, 1)
            self.pose = pose_stamped
            self._im_server.insert(gripper_marker, feedback_cb=self.handle_feedback)
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
        # self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_marker_node')  # TODO I don't remember if this name matters
    rospy.sleep(0.5)
    im_server = InteractiveMarkerServer('gripper_im_server',
                                        q_size=2)  # set q_size for running on real robot TODO may need to uncomment for sim
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)

    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    teleop = GripperTeleop(arm, gripper, im_server)
    # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    rospy.spin()


if __name__ == '__main__':
    main()
