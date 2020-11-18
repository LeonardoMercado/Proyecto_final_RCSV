#!/usr/bin/python

import rospy

import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 
from geometry_msgs.msg import TransformStamped

import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg

class Listener:
    
    # def __init__(self):
    def zx(self):

        # Topics
        self.topicPub = "/mobile_base/commands/velocity"
        # Para realizar la escucha "listener"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.trans_base_carrot = TransformStamped()

        # Publisher
        self.kobuki_vel = rospy.Publisher(self.topicPub , geometry_msgs.msg.Twist, queue_size=1)
        #--------------------------------------------------------------------------------------#
        # Polling Velocity to kobuki/ Twist Msg
        # rate = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        # /base_footprint /carrot1
        try:
            self.trans_base_carrot = self.tfBuffer.lookup_transform("base_footprint",
                                                                    "carrot1",
                                                                    rospy.Time.now(),rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform ")
            # return
            # rate.sleep()
            # continue

        # msg = geometry_msgs.msg.Twist()

        # msg.angular.z = 1.9 * math.atan2(self.trans.transform.translation.y, self.trans.transform.translation.x)
        # msg.linear.x = 0.05 * math.sqrt(self.trans.transform.translation.x ** 2 + self.trans.transform.translation.y ** 2)

        # self.kobuki_vel.publish(msg)
        # rate.sleep()
        # print(trans.transform.translation.x)
        return self.trans_base_carrot

    def movKobuki(self):
        
        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 1.9 * math.atan2(self.trans.transform.translation.y, self.trans.transform.translation.x)
        msg.linear.x = 0.05 * math.sqrt(self.trans.transform.translation.x ** 2 + self.trans.transform.translation.y ** 2)

        self.kobuki_vel.publish(msg)

    # def calcMTH(self):
        # Polling Velocity to kobuki/ Twist Msg
        # rate = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        # try:
        #     self.trans = self.tfBuffer.lookup_transform('base_footprint','carrot1', rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn("Error trying to look for transform calcMTH")
        #     # rate.sleep()
        #     # continue

        # return self.trans.transform.translation.x    