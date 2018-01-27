#! /usr/bin/env python

import rospy
import copy
import pickle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header

current_position = None
FILE_NAME = "savedPoses.p"


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def printCommands():
    print "Commands:"
    print "\tlist: List saved poses."
    print "\tsave <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists."
    print "\tdelete <name>: Delete the pose given by <name>."
    print "\tgoto <name>: Sends the robot to the pose given by <name>."
    print "\thelp: Show this list of commands"


def printPose(stampedPose):
    pose = stampedPose.pose.pose

    print "Position: ", pose.position.x, pose.position.y, pose.position.z
    print "Quaternion: ", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w


def updateCurrentPose(msg):
    global current_position
    current_position = msg


def main():
    rospy.init_node('pose_saver')
    wait_for_time()

    goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    current_pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=updateCurrentPose)

    try:
        poseList = pickle.load(open(FILE_NAME, "rb"))
    except Exception as e:
        print e
        poseList = {}

    print "Welcome to the map annotator!"
    printCommands()

    while True:
        input = raw_input("> ")
        tokens = input.split(" ", 1)
        command = tokens[0]

        if command == "list":
            for key, value in poseList.items():
                print key
                printPose(value)
        elif command == "save":
            poseList[tokens[1]] = current_position
        elif command == "delete":
            if tokens[1] in poseList.keys():
                del poseList[tokens[1]]
            else:
                print "No such pose", tokens[1]
        elif command == "goto":
            if tokens[1] in poseList.keys():
                newGoal = PoseStamped()
                newGoal.header = Header()
                newGoal.header.frame_id = "map"
                newGoal.pose = Pose()
                newGoal.pose = copy.deepcopy(poseList[tokens[1]].pose.pose)

                goto_publisher.publish(newGoal)
            else:
                print "No such pose", tokens[1]
        elif command == "help":
            printCommands()
        elif command == "quit":
            pickle.dump(poseList, open(FILE_NAME, "wb"))
            return
        else:
            print "Unrecognized input"
            printCommands()


if __name__ == '__main__':
    main()
