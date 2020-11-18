#!/usr/bin/python
import rospy

from kobuki_broadcaster_class import Broadcaster

# Init of program
if __name__ == '__main__':

    rospy.init_node('dynamic_tf2_broadcaster')
    rospy.loginfo("Node trajectory init")

    Broadcaster()

    rospy.spin()