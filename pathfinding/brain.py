def main():
    rospy.init_node('map_annotator_server')
    wait_for_time()

    server = PoseServer()

    rospy.spin()


if __name__ == '__main__':
    main()
