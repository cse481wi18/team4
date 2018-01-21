#!/usr/bin/env python


import math

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LOOK_AT_ACTION_NAME = 'head_controller/point_head'
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """

    # Values found here: http://docs.fetchrobotics.com/robot_hardware.html#joint-limits-and-types
    MIN_PAN = - 0.5 * math.pi  # Minimum pan angle, in radians.
    MAX_PAN = 0.5 * math.pi  # Maximum pan angle, in radians.
    MIN_TILT = - 0.25 * math.pi  # Minimum tilt angle, in radians.
    MAX_TILT = 0.5 * math.pi  # Maximum tilt angle, in radians.

    MIN_DURATION = rospy.Duration(1)

    def __init__(self):
        # Create actionlib clients
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        self.look_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)

        # Wait for both servers
        self.pan_tilt_client.wait_for_server()
        self.look_client.wait_for_server()

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # Create goal
        goal = PointHeadGoal()
        # Fill out the goal
        goal.min_duration = self.MIN_DURATION
        goal.max_velocity = 1.57  # From documentation: http://docs.fetchrobotics.com/robot_hardware.html#forces-and-torques
        head = Header()
        head.frame_id = frame_id

        goal.target = PointStamped(head, Point(x, y, z))

        # Send the goal & wait for result
        self.look_client.send_goal_and_wait(goal)

    def pan_tilt(self, pan, tilt):
        rospy.loginfo("Pan_tilt values: " + str(pan) + " " + str(tilt))
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # check that the pan/tilt angles are within joint limits
        if pan < self.MIN_PAN or pan > self.MAX_PAN or \
                tilt < self.MIN_TILT or tilt > self.MAX_TILT:
            return

        # Create a trajectory point
        trajectory_point = JointTrajectoryPoint()
        # Set positions of the two joints in the trajectory point
        trajectory_point.positions = [pan, tilt]
        # Set time of the trajectory point
        trajectory_point.time_from_start = rospy.Duration(PAN_TILT_TIME)

        # Create goal
        goal = FollowJointTrajectoryGoal()

        trajectory = JointTrajectory()
        # Add joint names to the list
        trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        # Add trajectory point created above to trajectory
        trajectory.points = [trajectory_point]

        goal.trajectory = trajectory

        # Send the goal & wait for result
        self.pan_tilt_client.send_goal_and_wait(goal)
