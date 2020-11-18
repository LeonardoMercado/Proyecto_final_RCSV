#!/usr/bin/python

import rospy
from one_class import Broadcaster_Listener

# Init of program
if __name__ == '__main__':

    rospy.init_node('one_broad_listener', anonymous=True)

    rospy.loginfo("Node init broad_listener")

    Broadcaster_Listener()

    rospy.spin()