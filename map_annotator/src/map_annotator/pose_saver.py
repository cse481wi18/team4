import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

current_position = None

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

def main():
    rospy.init_node('pose_saver')
    wait_for_time()

    goto_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    current_pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self._odom_callback)

    poseList = {}

    print "Welcome to the map annotator!"
    printCommands()

    while True:
        input = raw_input("> ")
        command = input.split()[0]

        if command == "list":
            for key, value in poseList.items():
                print key
                printPose(value)
        elif command == "save":
            currentPosition = current_pose_subscriber
        elif command == "delete":

        elif command == "goto":

        elif command == "help":
            printCommands()
        else:
            print "Unrecognized input"
            printCommands()

if __name__ == '__main__':

    main()
