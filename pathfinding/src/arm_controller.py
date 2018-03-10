# TODO milestone 1 implement class

import fetch_api
import pickle
import rospy
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from joint_state_reader import reader
import moveit_commander
import sys



GRIPPER_MARKER_OFFSET = 0.166
OBJECT_GRIPPER_OFFSET = - 0.04

GRIPPER_NAMES = ['l_gripper_finger_joint', 'r_gripper_finger_joint']

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
# TODO Lol
# /move_group_commander_wrappers_1520711652741721187 :89: Loading robot model 'fetch'...
# /move_group_commander_wrappers_1520711652741721187 :932: No root/virtual joint specified in SRDF. Assuming fixed joint
# /move_group_commander_wrappers_1520711652741721187 :89: Loading robot model 'fetch'...
# /move_group_commander_wrappers_1520711652741721187 :932: No root/virtual joint specified in SRDF. Assuming fixed joint
# /move_group_commander_wrappers_1520711652741721187 treeFromUrdfModel:197: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
# Traceback (most recent call last):
#   File "/home/team4/catkin_ws/src/cse481wi18/pathfinding/src/brain.py", line 143, in <module>
#     main()
#   File "/home/team4/catkin_ws/src/cse481wi18/pathfinding/src/brain.py", line 75, in main
#     my_arm = arm_controller.ArmController()
#   File "/home/team4/catkin_ws/src/cse481wi18/pathfinding/src/arm_controller.py", line 57, in __init__
#     self._group = moveit_commander.MoveGroupCommander('arm')
#   File "/opt/ros/indigo/lib/python2.7/dist-packages/moveit_commander/move_group.py", line 51, in __init__
#     self._g = _moveit_move_group_interface.MoveGroup(name, robot_description)
# RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)
# terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
#   what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
# Aborted (core dumped)



class ArmController:
    def __init__(self):
        pass
        # moveit_commander.roscpp_initialize(sys.argv)
        # self._arm = fetch_api.Arm()
        # self._gripper = fetch_api.Gripper()
        # self._gripper_state = reader.JointStateReader()
        # moveit_robot = moveit_commander.RobotCommander() #? need this?
        # self._group = moveit_commander.MoveGroupCommander('arm')
        # rospy.on_shutdown(self._on_shutdown) # stop moving on shutdown

        try:
            self.tuck_path = pickle.load(open("tuck_path.p", "rb"))
        except Exception as e:
            print e

        try:
            self.drop_path = pickle.load(open("drop_path.p", "rb"))
        except Exception as e:
            print e

        try:
            self.pick_path = pickle.load(open("pick_path.p", "rb"))
        except Exception as e:
            print e

    def ball_reachable(self, ball_pose):
        return self._path_ok(self.pick_path, ball_pose)

    # TODO is it possible that plan_arm will return true but straight_arm_move_to_pose wont?
    def _path_ok(self, path, ball_pose):
        print "[arm_controller: checking arm path...]"
        for pose, relative_to_ball, gripper_open in path:
            print "[arm_controller: checking next step]"
            ps = PoseStamped()
            ps.header.frame_id = 'base_link'

            if not relative_to_ball:
                ps.pose = Pose()
                ps.pose.position.x = pose.position[0]
                ps.pose.position.y = pose.position[1]
                ps.pose.position.z = pose.position[2]
                ps.pose.orientation.x = pose.orientation[0]
                ps.pose.orientation.y = pose.orientation[1]
                ps.pose.orientation.z = pose.orientation[2]
                ps.pose.orientation.w = pose.orientation[3]
            else:
                (point_with_position, point_with_quaternion) = pose.position, pose.orientation
                point_to_wrist_matrix = tft.quaternion_matrix(
                    [point_with_quaternion.x, point_with_quaternion.y, point_with_quaternion.z,
                     point_with_quaternion.w])
                point_to_wrist_matrix[:, 3] = (point_with_position.x, point_with_position.y, point_with_position.z, 1)
                base_to_point_matrix = tft.quaternion_matrix(
                    [ball_pose.orientation.x, ball_pose.orientation.y, ball_pose.orientation.z,
                     ball_pose.orientation.w])
                base_to_point_matrix[:, 3] = (ball_pose.position.x, ball_pose.position.y, ball_pose.position.z, 1)
                base_to_wrist_matrix = np.dot(base_to_point_matrix, point_to_wrist_matrix)
                ps.pose = Pose()
                ps.pose.position = Point(base_to_wrist_matrix[0, 3], base_to_wrist_matrix[1, 3],
                                         base_to_wrist_matrix[2, 3])
                temp = tft.quaternion_from_matrix(base_to_wrist_matrix)
                ps.pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])

            err = self._arm.check_pose(ps)
            if err is not None:
                return False
        return True

    # block & return upon arm tuck
    def execute_path(self, path, ball_pose):

        if not self._path_ok(path, ball_pose):
            print "[arm_controller: path not possible]"
            return False
        print "[arm_controller: executing arm path...]"
        # print "ball pose: ", ball_pose
        for pose, relative_to_ball, gripper_open in path:
            print "[arm_controller: executing next step]"
            if gripper_open:
                self._gripper.open()
            else:
                self._gripper.close()

            ps = PoseStamped()
            ps.header.frame_id = 'base_link'

            if not relative_to_ball:
                ps.pose = Pose()
                ps.pose.position.x = pose.position[0]
                ps.pose.position.y = pose.position[1]
                ps.pose.position.z = pose.position[2]
                ps.pose.orientation.x = pose.orientation[0]
                ps.pose.orientation.y = pose.orientation[1]
                ps.pose.orientation.z = pose.orientation[2]
                ps.pose.orientation.w = pose.orientation[3]
                err = self._arm.straight_move_to_pose(self._group, ps, jump_threshold=2.0)
                if err is not None:
                    print "[arm_controller: path execution failed]"
                    rospy.logerr(err)
                    return False
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
                err = self._arm.straight_move_to_pose(self._group, ps, jump_threshold=2.0)
                if err is not None:
                    print "[arm_controller: path execution failed]"
                    rospy.logerr(err)
                    return False
            rospy.sleep(0.5)  # let the arm finish moving to prevent CONTROL_FAILED errors
        return True

    def tuck_arm(self):
        print "[arm_controller: tucking arm...]"
        return self.execute_path(self.tuck_path, None)

    def pick_up_ball(self, ball_pose):
        self._gripper.open()
        possible = self.execute_path(self.pick_path, ball_pose)
        if not possible:
            rospy.sleep(1.0)
            self.tuck_arm()
            return False
        print "[arm_controller: getting gripper state]"
        values = self._gripper_state.get_joints(GRIPPER_NAMES)
        for value in values:
            if value < 0.01:
                return False
        return True

    def drop_ball_in_basket(self):
       part1 = self.execute_path(self.drop_path, None)
       part2 = self.execute_path(self.tuck_path, None)
       return part1 and part2

    def _on_shutdown(self):
        self._group.stop()
        moveit_commander.roscpp_shutdown()
