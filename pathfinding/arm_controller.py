# TODO milestone 1 implement class

import fetch_api
import copy
from geometry_msgs.msg import PoseStamped, Pose, Point

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

def ArmController:
    def __init__(self):
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()

    # block & return upon arm tuck
    def tuck_arm(self):
        # blocking
        pass

    # ball pose type TBD
    # returns True if ball pickup is successful
    def pick_up_ball(self, ball_pose):
        pre_pose = Pose()
        pre_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET - 0.1

        pre_base_pose = PoseStamped()
        pre_base_pose.header.frame_id = "base_link"
        pre_base_pose.pose = transform_gripper_to_base(pre_pose, ball_pose.pose)

        grasp_pose = Pose()
        grasp_pose.position.x = -GRIPPER_MARKER_OFFSET + OBJECT_GRIPPER_OFFSET

        grasp_base_pose = PoseStamped()
        grasp_base_pose.header.frame_id = "base_link"
        grasp_base_pose.pose = transform_gripper_to_base(grasp_pose, ball_pose.pose)

        self._gripper.open()
        self._arm.move_to_pose(pre_base_pose)
        self._arm.move_to_pose(grasp_base_pose)
        self._gripper.close()
        self.tuck_arm()
        # blocking
        return False

    # go through set list of poses for gripper + arm\
    # look at disco/arm wave demo
    def drop_ball_in_basket(self):
        # blocking
        return False
