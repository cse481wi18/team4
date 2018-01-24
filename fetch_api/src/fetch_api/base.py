#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft
import numpy as np

import copy
import math


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Publish stuff
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Subscribe to odometry info
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self.has_received_odom_msg = False
        pass

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist msg
        newMsg = Twist()
        # Fill out msg
        newMsg.linear.x = linear_speed
        newMsg.linear.y = 0.0
        newMsg.linear.z = 0.0
        newMsg.angular.x = 0.0
        newMsg.angular.y = 0.0
        newMsg.angular.z = angular_speed
        # Publish msg
        self.pub.publish(newMsg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        newMsg = Twist()
        newMsg.linear.x = 0.0
        newMsg.linear.y = 0.0
        newMsg.linear.z = 0.0
        newMsg.angular.x = 0.0
        newMsg.angular.y = 0.0
        newMsg.angular.z = 0.0
        self.pub.publish(newMsg)

    def _odom_callback(self, msg):
        """
        :param msg: nav_msgs/Odometry Message
        """
        self.has_received_odom_msg = True
        self.last_position = msg.pose.pose  # contains 'position' and 'orientation'

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while not self.has_received_odom_msg:
            rospy.sleep(0.5)

        # record start position, use Python's copy.deepcopy
        start_pos = copy.deepcopy(self.last_position.position)
        rate = rospy.Rate(10)
        # CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        current_position = copy.deepcopy(self.last_position.position)
        traveled_distance = math.sqrt(math.pow((current_position.x - start_pos.x), 2) +
                                      math.pow((current_position.y - start_pos.y), 2) +
                                      math.pow((current_position.z - start_pos.z), 2))
        while (not rospy.is_shutdown() and traveled_distance < math.fabs(distance)):
            current_position = copy.deepcopy(self.last_position.position)
            traveled_distance = math.sqrt(math.pow((current_position.x - start_pos.x), 2) +
                                          math.pow((current_position.y - start_pos.y), 2) +
                                          math.pow((current_position.z - start_pos.z), 2))
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            if traveled_distance >= math.fabs(distance):
                break
            rate.sleep()

    def turn(self, angular_distance, angular_speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        while not self.has_received_odom_msg:
            rospy.sleep(0.5)

        # record start position
        start_orientation = copy.deepcopy(
            self.last_position.orientation)  # *shudders* it's a Quaternion, not sure how to use
        start_yaw = self.quaternion_to_yaw(start_orientation)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?

        # TODO do we even care if angular_distance is < or > 2pi? shouldn't we just spin around that many times?
        ##TODO  Yeah - we lost that info on converting to radians. e.g. 370 degrees should spin more than 1 time, but
        ## is considered equivalent to 10 degrees.
        polarity = -1 if angular_distance < 0 else 1
        finish_angle = angular_distance % (2 * math.pi) * polarity
        current_orientation = copy.deepcopy(self.last_position.orientation)
        # Idea: get yaw (rotation abt z azis)
        current_yaw = self.quaternion_to_yaw(current_orientation)
        traveled_angle = math.fabs(current_yaw - start_yaw)

        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        while (not rospy.is_shutdown() and traveled_angle < math.fabs(finish_angle)):
            # TODO need to calculate how much the angle has changed, need to deal with "wraparound" issue
            current_orientation = copy.deepcopy(self.last_position.orientation)
            current_yaw = self.quaternion_to_yaw(current_orientation)
            traveled_angle = math.fabs(current_yaw - start_yaw)
            angular_speed = max(0.25, min(1, angular_speed))
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * angular_speed)
            rate.sleep()

    def quaternion_to_yaw(self, q):
        rotation_matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = rotation_matrix[0, 0]
        y = rotation_matrix[1, 0]
        theta_rads = math.atan2(y, x)
        return theta_rads
