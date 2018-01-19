#! /usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
import rospy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
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
