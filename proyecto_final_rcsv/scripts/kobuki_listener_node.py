#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math
import numpy as np
from kobuki_listener_class import Listener


if __name__ == '__main__':
    # Node things
    
    rospy.init_node('dynamic_tf2_listener')
    rospy.loginfo("Node listener init")

    # Calling the class
    Listener()

    rospy.spin()

