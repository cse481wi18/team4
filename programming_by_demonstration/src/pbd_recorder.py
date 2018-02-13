import query_action_client
import fetch_api
import rospy
import tf.transformations as tft
import numpy as np
import pickle
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
        ps.header.frame_id = 'base_link'
        if frame is not -1:
            # transform
            wrist_matrix = pose_to_matrix(ps.pose)
            tag1_matrix = pose_to_matrix(self.reader.markers[frame])

            pose = matrix_to_pose(np.dot(np.linalg.inv(tag1_matrix), wrist_matrix))
            ps.pose = pose
        self.stamped_poses.append((ps, frame))

    # Saving program
    def save_path(self, name):
        rospy.logerr("Writing pickle file")
        try:
            pickle.dump(self.stamped_poses, open(name + ".p", "wb"))
            rospy.logerr("Dumped pickle file!")
        except Exception as e:
            rospy.roserr(e)
        finally:
            self.stamped_poses = []

    def get_tags(self):
        tag_list = []
        for marker in self.reader.markers.keys():
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
            for pose, frame in path:
                if frame is -1:
                    rospy.logerr('moving to pose')
                    err = self.arm.move_to_pose(pose)
                else:
                    rospy.logerr('moving to pose relative to tag')
                    tag_to_base = pose_to_matrix(self.reader.markers[frame])
                    wrist_to_tag = pose_to_matrix(pose.pose)
                    goal = matrix_to_pose(np.dot(wrist_to_tag, tag_to_base))
                    pose.pose = goal
                    rospy.logerr('sending goal')
                    err = self.arm.move_to_pose(pose)
                    rospy.logerr('goal executed')
                if err is not None:
                    print err


class ArTagReader():
    def __init__(self):
        self.markers = {}

    def callback(self, msg):
        self.markers = {}
        markers = msg.markers
        for m in markers:
            self.markers[m.id] = m.pose.pose
        # for m in markers:
        #     print msg
