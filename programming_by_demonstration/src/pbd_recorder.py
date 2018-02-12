import query_action_client


class Recorder:
    def __init__(self):
        self.stamped_poses = []
        self.arm_controller = query_action_client.arm_control()

    def arm_limp(self):
        self.arm_controller.stop_arm()

    def arm_rigid(self):
        self.arm_controller.start_arm()

    # frame: (int) marker_id -- assuming it's always the same, -1 for base_frame
    def record_pose(self, frame):
        pass

    def save_path(self, name):
        pass

    def get_tags(self):
        pass
