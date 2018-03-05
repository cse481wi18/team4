# TODO milestone 1 implement class

import fetch_api
import pickle
import rospy
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

GRIPPER_MARKER_OFFSET = 0.166
OBJECT_GRIPPER_OFFSET = - 0.04

def transform_gripper_to_base(gripperPose, objectPose):
    """self.pose_ok
    transform_base_frame_to_gripper_frame
    :param pose:
    :return:
    """
    gripperMatrix = pose_to_matrix(gripperPose)
    objectMatrix = pose_to_matrix(objectPose)

    pose = np.dot(objectMatrix, gripperMatrix)

    return matrix_to_pose(pose)

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

class ArmController:
    def __init__(self):
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()

        # TODO not block
        # try:
        #     self.tuck_path = pickle.load(open("tuck_path.p", "rb"))
        # except Exception as e:
        #     print e
        #
        # try:
        #     self.drop_path = pickle.load(open("drop_path.p", "rb"))
        # except Exception as e:
        #     print e
        #
        # try:
        #     self.pick_path = pickle.load(open("pick_path.p", "rb"))
        # except Exception as e:
        #     print e

    # block & return upon arm tuck
    def execute_path(self, path, ball_pose):
        for pose, relative_to_ball, gripper_open in path:
            if gripper_open:
                self._gripper.open()
            else:
                self._gripper.close()

            ps = PoseStamped()
            ps.header.frame_id = 'base_link'

            if not relative_to_ball:
                ps.pose = pose
                err = self._arm.move_to_pose(ps)
                if err is not None:
                    rospy.logerr(err)
            else:
                (point_with_position, point_with_quaternion) = pose.position, pose.orientation
                point_to_wrist_matrix = tft.quaternion_matrix(
                    [point_with_quaternion.x, point_with_quaternion.y, point_with_quaternion.z, point_with_quaternion.w])
                point_to_wrist_matrix[:, 3] = (point_with_position.x, point_with_position.y, point_with_position.z, 1)
                base_to_point_matrix = tft.quaternion_matrix(
                    [ball_pose.orientation.x, ball_pose.orientation.y, ball_pose.orientation.z, ball_pose.orientation.w])
                base_to_point_matrix[:, 3] = (ball_pose.position.x, ball_pose.position.y, ball_pose.position.z, 1)
                base_to_wrist_matrix = np.dot(base_to_point_matrix, point_to_wrist_matrix)
                ps.pose = Pose()
                ps.pose.position = Point(base_to_wrist_matrix[0, 3], base_to_wrist_matrix[1, 3], base_to_wrist_matrix[2, 3])
                temp = tft.quaternion_from_matrix(base_to_wrist_matrix)
                ps.pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])

                err = self._arm.move_to_pose(ps)
                if err is not None:
                    rospy.logerr(err)

    def tuck_arm(self):
        self.execute_path(self.tuck_path, None)

    def pick_up_ball(self, ball_pose):
        self.execute_path(self.pick_path, ball_pose)

    # ball pose type TBD
    # block before return
    # def pick_up_ball(self, ball_pose):
    #     pre_pose = Pose()
    #     pre_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET - 0.1
    #
    #     pre_base_pose = PoseStamped()
    #     pre_base_pose.header.frame_id = "base_link"
    #     pre_base_pose.pose = transform_gripper_to_base(pre_pose, ball_pose.pose)
    #
    #     grasp_pose = Pose()
    #     grasp_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET
    #
    #     grasp_base_pose = PoseStamped()
    #     grasp_base_pose.header.frame_id = "base_link"
    #     grasp_base_pose.pose = transform_gripper_to_base(grasp_pose, ball_pose.pose)
    #
    #     self._gripper.open()
    #     self._arm.move_to_pose(pre_base_pose)
    #     self._arm.move_to_pose(grasp_base_pose)
    #     self._gripper.close()
    #     self.tuck_arm()
    #     # blocking
    #     return False

    # go through set list of poses for gripper + arm\
    # look at disco/arm wave demo
    def drop_ball_in_basket(self):
       self.execute_path(self.drop_path, None)
