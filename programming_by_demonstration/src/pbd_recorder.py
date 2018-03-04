import query_action_client
import fetch_api
import rospy
import tf.transformations as tft
import tf
import numpy as np
import pickle
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point


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


class Recorder:
    def __init__(self):
        self.stamped_poses = []
        self._poses = []
        self._arm_controller = query_action_client.arm_control()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._reader = ArTagReader()
        self._sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers,
                                     self._reader.callback)  # Subscribe to AR tag poses, use reader.callback
        self._transform_listener = tf.TransformListener()

    def arm_limp(self):
        self._arm_controller.stop_arm()

    def arm_rigid(self):
        self._arm_controller.start_arm()

    # frame: (int) marker_id -- assuming it's always the same, -1 for base_frame
    def record_pose(self, relative_to_ball, ball_pose = None, gripper_open=True):
        pose = Pose()
        if not relative_to_ball:
            (position, quaternion) = self._transform_listener.lookupTransform('base_link', 'wrist_roll_link',
                                                                              rospy.Time(0))
            pose.position = position
            pose.orientation = quaternion
            self._poses.append((pose, frame, gripper_open))
        else:
            curr_marker = ball_pose
            (pos_b, quat_b) = self._transform_listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
            base_to_wrist_matrix = tft.quaternion_matrix(quat_b)
            pos_b.append(1)
            base_to_wrist_matrix[:, 3] = pos_b
            base_to_tag_matrix = tft.quaternion_matrix(
                [curr_marker.orientation.x, curr_marker.orientation.y, curr_marker.orientation.z,
                 curr_marker.orientation.w])
            base_to_tag_matrix[:, 3] = (curr_marker.position.x, curr_marker.position.y, curr_marker.position.z, 1)
            t_b_matrix = np.linalg.inv(base_to_tag_matrix)
            tag_to_wrist_matrix = np.dot(t_b_matrix, base_to_wrist_matrix)
            pose.position = Point(tag_to_wrist_matrix[0, 3], tag_to_wrist_matrix[1, 3], tag_to_wrist_matrix[2, 3])
            temp = tft.quaternion_from_matrix(tag_to_wrist_matrix)
            pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])
            self._poses.append((pose, frame, gripper_open))

    # Saving program
    def save_path(self, name):
        rospy.logerr("Writing pickle file")
        try:
            # pickle.dump(self.stamped_poses, open(name + ".p", "wb"))
            pickle.dump(self._poses, open(name + ".p", "wb"))
            rospy.logerr("Dumped pickle file!")
        except Exception as e:
            rospy.roserr(e)
        finally:
            self.stamped_poses = []
            self._poses = []

    def get_tags(self):
        tag_list = []
        for marker in self._reader.markers.keys():
            tag_list.append(marker)
        return tag_list

    def execute_path(self, name):
        path = []

        try:
            path = pickle.load(open(name + ".p", "rb"))
            rospy.logerr("Read from pickle file.")
        except Exception as e:
            print e
        if len(path) > 0:
            for pose, frame, gripper_open in path:
                print "pose: {}, frame: {}, gripper_open: {}".format(pose, frame, gripper_open)
                if gripper_open:
                    self._gripper.open()
                else:
                    self._gripper.close()

                ps = PoseStamped()
                ps.header.frame_id = 'base_link'

                if frame == -1:
                    ps.pose = Pose()
                    ps.pose.position.x = pose.position[0]
                    ps.pose.position.y = pose.position[1]
                    ps.pose.position.z = pose.position[2]
                    ps.pose.orientation.x = pose.orientation[0]
                    ps.pose.orientation.y = pose.orientation[1]
                    ps.pose.orientation.z = pose.orientation[2]
                    ps.pose.orientation.w = pose.orientation[3]

                    error = self._arm.move_to_pose(ps)
                    if error is not None:
                        rospy.logerr(error)
                else:
                    curr_marker = self._reader.markers[frame]
                    (tag_with_position, tag_with_quaternion) = pose.position, pose.orientation
                    tag_to_wrist_matrix = tft.quaternion_matrix(
                        [tag_with_quaternion.x, tag_with_quaternion.y, tag_with_quaternion.z, tag_with_quaternion.w])
                    tag_to_wrist_matrix[:, 3] = (tag_with_position.x, tag_with_position.y, tag_with_position.z, 1)
                    base_to_tag_matrix = tft.quaternion_matrix(
                        [curr_marker.orientation.x, curr_marker.orientation.y, curr_marker.orientation.z,
                         curr_marker.orientation.w])
                    base_to_tag_matrix[:, 3] = (
                        curr_marker.position.x, curr_marker.position.y, curr_marker.position.z, 1)
                    base_to_wrist_matrix = np.dot(base_to_tag_matrix, tag_to_wrist_matrix)
                    ps.pose = Pose()
                    ps.pose.position = Point(base_to_wrist_matrix[0, 3], base_to_wrist_matrix[1, 3],
                                             base_to_wrist_matrix[2, 3])
                    temp = tft.quaternion_from_matrix(base_to_wrist_matrix)
                    ps.pose.orientation = Quaternion(temp[0], temp[1], temp[2], temp[3])

                    error = self._arm.move_to_pose(ps)
                    if error is not None:
                        rospy.logerr(error)


class ArTagReader():
    def __init__(self):
        self.markers = {}
        # self.has_stored_markers = False

    def callback(self, msg):
        markers = {}  # TODO - figure out handling not seeing AR markers - cant move robot
        copied_markers = msg.markers
        for m in copied_markers:
            markers[m.id] = m.pose.pose
            # if not self.has_stored_markers:
            self.markers = markers
            # self.has_stored_markers = True
        # for m in self.markers:
        #     print msg
