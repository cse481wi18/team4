#!/usr/bin/env 


import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import rospy

# TODO: ACTION_NAME = ???
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # TODO: Wait for server
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if not (height < Torso.MIN_HEIGHT or height > Torso.MAX_HEIGHT):
            # TODO: Create a trajectory point
            message = JointTrajectory()
            # TODO: Add joint name to list
            message.joint_names = [JOINT_NAME]
            # TODO: Set position of trajectory point
            point = JointTrajectoryPoint()
            point.positions = [height]
            # TODO: Set time of trajectory point
            point.time_from_start = rospy.Duration(TIME_FROM_START)
            message.points = [point]
            # TODO: Create goal
            goal = FollowJointTrajectoryGoal()
            # TODO: Add the trajectory point created above to trajectory
            goal.trajectory = message
            # TODO: Send goal
            self.client.send_goal(goal)
            # TODO: Wait for result
            self.client.wait_for_result()
