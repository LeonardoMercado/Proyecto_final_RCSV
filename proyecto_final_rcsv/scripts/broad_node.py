#!/usr/bin/python

import rospy
from broadcaster_class import Broadcaster

# Init of program
if __name__ == '__main__':

    rospy.init_node('kobuki_tf2_broadcaster', anonymous=True)

    rospy.loginfo("Node init")

    Broadcaster()

    rospy.spin()