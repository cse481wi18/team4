#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import math


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class NavPath(object):
    def __init__(self):
        self._path = []
        inital = Point(0.0, 0.0, 0.0)
        self._path.append(inital)
        self.id = -1
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def callback(self, msg):
        new_pos = msg.pose.pose.position
        n_x = new_pos.x
        n_y = new_pos.y
        n_z = new_pos.z
        pos = self._path[len(self._path) - 1]
        x = pos.x
        y = pos.y
        z = pos.z
        has_moved = math.sqrt(math.pow((n_x - x), 2) + math.pow((n_y - y), 2) + math.pow((n_z - z), 2)) > 0.1
        if has_moved:
            self._path.append(msg.pose.pose.position)
            self.id += 1
            marker = Marker(
                type=Marker.SPHERE,
                id=self.id,
                lifetime=rospy.Duration(500),
                pose=Pose(Point(n_x, n_y, n_z), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.6, 0.6, 0.6),
                header=Header(frame_id='odom'),
                color=ColorRGBA(1.0, 1.0, 0.0, 0.8)
            )
            self.marker_publisher.publish(marker)


def main():
    # ...setup stuff...
    rospy.init_node("some_node")
    nav_path = NavPath()
    rospy.sleep(0.5)
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    wait_for_time()
    rospy.spin()


if __name__ == '__main__':
    main()
