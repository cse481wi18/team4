import query_action_client
from joint_state_reader import JointStateReader


class recorder:
    def __init__(self):
        self.joint_state_reader = JointStateReader()
        self.stamped_poses = []
        self.arm_controller = query_action_client.arm_control()

    def arm_limp(self):
        self.arm_controller.stop_arm()

    def arm_rigid(self):
        self.arm_controller.start_arm()

    # frame: marker_id -- assuming it's always the same, -1 for base_frame
    def record_pose(self, frame):
        self.joint_state_reader.get_joints()

    def save_path(self, name):
        pass
