# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TIME_FROM_START = 5


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # Create actionlib client
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        # Wait for server
        self.client.wait_for_server()
        pass

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        message = JointTrajectory()
        # Set position of trajectory point
        point = JointTrajectoryPoint()
        point.positions = arm_joints.values()
        # Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        message.points = [point]
        # Create goal
        goal = FollowJointTrajectoryGoal()
        # Add joint name to list
        message.joint_names = arm_joints.names()
        # Add the trajectory point created above to trajectory
        goal.trajectory = message

        # Send goal
        self.client.send_goal(goal)
        # Wait for result
        self.client.wait_for_result()
