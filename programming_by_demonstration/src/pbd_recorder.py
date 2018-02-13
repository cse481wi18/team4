import query_action_client
import fetch_api
import rospy
import tf.transformations as tft
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, PoseStamped


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
        self.arm_controller = query_action_client.arm_control()
        self.arm = fetch_api.Arm()
        self.reader = ArTagReader()
        self.sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers,
                                    self.reader.callback)  # Subscribe to AR tag poses, use reader.callback

    def arm_limp(self):
        self.arm_controller.stop_arm()

    def arm_rigid(self):
        self.arm_controller.start_arm()

    # frame: (int) marker_id -- assuming it's always the same, -1 for base_frame
    def record_pose(self, frame):
        ps = PoseStamped()
        ps.pose = self.arm.get_pose()
        if frame == -1:
            ps.header.frame_id = 'base_link'
        else:
            # transform
            wrist_matrix = ps.pose
            # tag1_matrix =
            # np.dot()
        self.stamped_poses.append(ps)

    # Saving program
    def save_path(self, name):
        pass

    def get_tags(self):
        tag_list = []
        for marker in self.reader.markers:
            tag_list.append(marker.id)
        return tag_list

    def read_tags(self):
        pass


class ArTagReader():
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = []
        markers = msg.markers
        self.markers.extend(markers)
        # for m in markers:
        #     print msg
