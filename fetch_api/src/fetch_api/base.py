#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        :return:
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

        # TODO: record start position, use Python's copy.deepcopy
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
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            if traveled_distance >= math.fabs(distance):
                break
            rate.sleep()
