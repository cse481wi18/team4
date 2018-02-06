#! /usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from std_msgs.msg import ColorRGBA
import fetch_api
import rospy
import numpy as np
import copy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Pose, Point

import sys

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

MENU_GRIP_OPEN = 1
MENU_GRIP_CLOSE = 2
MENU_GRIP_MOVE = 3

MENU_GO = 1

"""
Notes from Justin:
only track position of the "interactive marker," the meshes are defined relative to the interactive marker -
don't set the poses of the meshes
Don't set the frame-ids of each mesh

For the autopick, have 3 gripper poses in frame of object, to convert to base frame:
multiply transform of base to object by transform of object to gripper to get transform from base to gripper
(bTo * oTg = bTg)

"""

GRIPPER_MARKER_OFFSET = 0.166
OBJECT_GRIPPER_OFFSET = - 0.04
OBJECT_LIFT_OFFSET = 0.1


def pose_to_matrix(pose):
    transformationMatrix = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                                  pose.orientation.w])
    transformationMatrix[:, 3] = (pose.position.x, pose.position.y, pose.position.z, 1)
    return transformationMatrix


def matrix_to_pose(matrix):
    pose = Pose()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w \
        = tft.quaternion_from_matrix(matrix)
    pose.position.x = matrix[0][3]
    pose.position.y = matrix[1][3]
    pose.position.z = matrix[2][3]

    return pose


def create_grippers_markers(gripper_interactive_marker):
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = True  # handle drag events

    # teal gripper marker
    gripper_body = Marker()
    gripper_body.type = Marker.MESH_RESOURCE
    gripper_body.mesh_resource = GRIPPER_MESH
    # gripper_body.pose.orientation.w = 1
    gripper_body.pose.position.x = GRIPPER_MARKER_OFFSET
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
    left_finger.pose.position.x = GRIPPER_MARKER_OFFSET
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
    right_finger.pose.position.x = GRIPPER_MARKER_OFFSET
    # right_finger.scale.x = 0.45 self.obj_marker.pose.position.x = 1
    # right_finger.scale.y = 0.45
    # right_finger.scale.z = 0.45
    right_finger.color = ColorRGBA(0, 0, 1, 1)
    control.markers.append(right_finger)

    gripper_interactive_marker.controls.append(control)  # dont remember if python is pass by value/ref


# Returns a list of InteractiveMarkerControls
def make_6of_controls():
    controls = []

    control_x = InteractiveMarkerControl()
    control_x.name = 'move_x'
    control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_x.always_visible = True
    # control_x.scale.x = 0.45
    # control_x.scale.y = 0.45
    # control_x.scale.z = 0.45
    controls.append(control_x)
    rot_control_x = copy.deepcopy(control_x)
    rot_control_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_control_x.name = 'rotate_x'
    rot_control_x.orientation.w = 1
    rot_control_x.orientation.x = 1
    rot_control_x.orientation.y = 0
    rot_control_x.orientation.z = 0
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

    def start(self):
        self.gripper_im = InteractiveMarker()
        self.gripper_im.header.frame_id = "base_link"
        self.gripper_im.name = "gripper_teleop_marker"
        self.gripper_im.description = "Gripper"
        self.gripper_im.pose.position.x = 1
        self.gripper_im.pose.position.y = 0
        self.gripper_im.pose.position.z = 0
        create_grippers_markers(self.gripper_im)
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
                    'num_planning_attempts': 10,
                    'replan': True,
                }
                if self.pose_ok:
                    self._arm.move_to_pose(self.pose, **kwargs)
                    self._im_server.insert(gripper_marker, feedback_cb=self.handle_feedback)
                    self._im_server.applyChanges()
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
        self._obj_marker = InteractiveMarker()
        self._obj_marker.header.frame_id = "base_link"
        self._obj_marker.name = "autopick_teleop_marker"
        self._obj_marker.description = "autopick_teleop_marker"
        self._obj_marker.pose.position.x = 1
        self._obj_marker.pose.position.y = 0
        self._obj_marker.pose.position.z = 0
        self.pose_ok = [False, False, False]
        self.poses = [None, None, None]

    def start(self):
        self.create_autopick_markers(self._obj_marker)
        print "created autopick"
        self._obj_marker.controls.extend(make_6of_controls())
        print "made controls"

        open_entry = MenuEntry()
        open_entry.id = MENU_GO
        open_entry.title = "Go"
        self._obj_marker.menu_entries.append(open_entry)

        self._im_server.insert(self._obj_marker, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def transform_gripper_to_base(self, gripperPose, objectPose):
        """self.pose_ok
        transform_base_frame_to_gripper_frame
        :param pose:
        :return:
        """
        gripperMatrix = pose_to_matrix(gripperPose)
        objectMatrix = pose_to_matrix(objectPose)

        pose = np.dot(objectMatrix, gripperMatrix)

        return matrix_to_pose(pose)

    def handle_feedback(self, feedback):
        gripper_marker = copy.deepcopy(self._im_server.get('autopick_teleop_marker'))
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == MENU_GO:
                kwargs = {
                    'allowed_planning_time': 10,
                    'execution_timeout': 10,
                    'num_planning_attempts': 10,
                    'replan': True,
                }
                if all(item for item in self.pose_ok):
                    self._gripper.open()
                    self._arm.move_to_pose(self.poses[0], **kwargs)

                    self._arm.move_to_pose(self.poses[1], **kwargs)
                    self._gripper.close()

                    self._arm.move_to_pose(self.poses[2], **kwargs)
                else:
                    rospy.logerr("Error - couldn't move to poses")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.logerr("Handling update pose")
            pose_stamped = PoseStamped()
            pose_stamped.pose = copy.deepcopy(feedback.pose)
            pose_stamped.header = copy.deepcopy(feedback.header)

            pre_gripper_pose = Pose()
            pre_gripper_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET - 0.1
            pre_base_pose = PoseStamped()
            pre_base_pose.header.frame_id = "base_link"
            pre_base_pose.pose = self.transform_gripper_to_base(pre_gripper_pose, gripper_marker.pose)
            self.poses[0] = pre_base_pose

            grasp_gripper_pose = Pose()
            grasp_gripper_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET
            grasp_base_pose = PoseStamped()
            grasp_base_pose.header.frame_id = "base_link"
            grasp_base_pose.pose = self.transform_gripper_to_base(grasp_gripper_pose, gripper_marker.pose)
            self.poses[1] = grasp_base_pose

            lift_gripper_pose = Pose()
            lift_gripper_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET
            lift_gripper_pose.position.z = OBJECT_LIFT_OFFSET
            lift_base_pose = PoseStamped()
            lift_base_pose.header.frame_id = "base_link"
            lift_base_pose.pose = self.transform_gripper_to_base(lift_gripper_pose, gripper_marker.pose)
            self.poses[2] = lift_base_pose

            result = self._arm.compute_ik(self.poses[0])
            self.pose_ok[0] = result

            for i in range(1, 4):
                mesh = gripper_marker.controls[0].markers[i]
                if result:
                    mesh.color = ColorRGBA(0, 1, 0, 1)
                else:
                    mesh.color = ColorRGBA(1, 0, 0, 1)

            result = self._arm.compute_ik(self.poses[1])
            self.pose_ok[1] = result

            for i in range(4, 7):
                mesh = gripper_marker.controls[0].markers[i]
                if result:
                    mesh.color = ColorRGBA(0, 1, 0, 1)
                else:
                    mesh.color = ColorRGBA(1, 0, 0, 1)

            result = self._arm.compute_ik(self.poses[2])
            self.pose_ok[2] = result

            for i in range(7, 10):
                mesh = gripper_marker.controls[0].markers[i]
                if result:
                    mesh.color = ColorRGBA(0, 1, 0, 1)
                else:
                    mesh.color = ColorRGBA(1, 0, 0, 1)

            self._im_server.insert(gripper_marker, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()

        pass

    def create_autopick_markers(self, gripper_interactive_marker):
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible = True  # handle drag events

        cube = Marker()
        cube.type = Marker.CUBE
        cube.color = ColorRGBA(1, 1, 0, 0.5)
        cube.scale.x = 0.05
        cube.scale.y = 0.05
        cube.scale.z = 0.05
        control.markers.append(cube)

        # Pre left/right fingers
        pre_left_finger = Marker()
        pre_left_finger.type = Marker.MESH_RESOURCE
        pre_left_finger.mesh_resource = L_FINGER_MESH
        pre_left_finger.pose.position.y = -.16
        pre_left_finger.pose.position.x = OBJECT_GRIPPER_OFFSET - 0.1
        pre_left_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(pre_left_finger)
        pre_right_finger = Marker()
        pre_right_finger.type = Marker.MESH_RESOURCE
        pre_right_finger.mesh_resource = L_FINGER_MESH
        pre_right_finger.pose.position.y = -.06
        pre_right_finger.pose.position.x = OBJECT_GRIPPER_OFFSET - 0.1
        pre_right_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(pre_right_finger)
        pre_gripper_body = Marker()
        pre_gripper_body.type = Marker.MESH_RESOURCE
        pre_gripper_body.mesh_resource = GRIPPER_MESH
        pre_gripper_body.pose.position.x = OBJECT_GRIPPER_OFFSET - 0.1
        pre_gripper_body.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(pre_gripper_body)

        # grab left/right fingers - other colors
        grab_left_finger = Marker()
        grab_left_finger.type = Marker.MESH_RESOURCE
        grab_left_finger.mesh_resource = L_FINGER_MESH
        grab_left_finger.pose.position.y = -.16
        grab_left_finger.pose.position.x = OBJECT_GRIPPER_OFFSET
        grab_left_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(grab_left_finger)
        grab_right_finger = Marker()
        grab_right_finger.type = Marker.MESH_RESOURCE
        grab_right_finger.mesh_resource = L_FINGER_MESH
        grab_right_finger.pose.position.y = -.06
        grab_right_finger.pose.position.x = OBJECT_GRIPPER_OFFSET
        grab_right_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(grab_right_finger)
        grab_gripper_body = Marker()
        grab_gripper_body.type = Marker.MESH_RESOURCE
        grab_gripper_body.mesh_resource = GRIPPER_MESH
        grab_gripper_body.pose.position.x = OBJECT_GRIPPER_OFFSET
        grab_gripper_body.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(grab_gripper_body)

        # lift left/right fingers - other colors
        lift_left_finger = Marker()
        lift_left_finger.type = Marker.MESH_RESOURCE
        lift_left_finger.mesh_resource = L_FINGER_MESH
        lift_left_finger.pose.position.y = -.16
        lift_left_finger.pose.position.z = OBJECT_LIFT_OFFSET
        lift_left_finger.pose.position.x = OBJECT_GRIPPER_OFFSET

        lift_left_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(lift_left_finger)
        lift_right_finger = Marker()
        lift_right_finger.type = Marker.MESH_RESOURCE
        lift_right_finger.mesh_resource = L_FINGER_MESH
        lift_right_finger.pose.position.y = -.06
        lift_right_finger.pose.position.z = OBJECT_LIFT_OFFSET
        lift_right_finger.pose.position.x = OBJECT_GRIPPER_OFFSET

        lift_right_finger.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(lift_right_finger)
        lift_gripper_body = Marker()
        lift_gripper_body.type = Marker.MESH_RESOURCE
        lift_gripper_body.mesh_resource = GRIPPER_MESH
        lift_gripper_body.pose.position.x = OBJECT_GRIPPER_OFFSET
        lift_gripper_body.pose.position.z = OBJECT_LIFT_OFFSET
        lift_gripper_body.color = ColorRGBA(0, 0, 1, 1)
        control.markers.append(lift_gripper_body)

        gripper_interactive_marker.controls.append(control)


def main():
    rospy.init_node('gripper_marker_node')
    rospy.sleep(0.5)
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    # im_server = InteractiveMarkerServer('gripper_im_server',
    #                                     q_size=2)  # set q_size for running on real robot TODO may need to uncomment for sim
    # teleop = GripperTeleop(arm, gripper, im_server)
    # teleop.start()

    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    auto_pick.start()
    rospy.spin()


if __name__ == '__main__':
    main()
