#!/usr/bin/python

import rospy

import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 
import time
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from kobuki_listener_class import Listener

class Broadcaster:
    
    def __init__(self):
        
        # Para realizar el "broadcast"
        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.trans = TransformStamped()

        # Trajectory
        i=0
        trajecx = np.array([-3.5,-3.5, 1.5, 1.5, 3.5, 3.5,-2.5,-2.5, 1.5, 1.5,-1.0])
        trajecy = np.array([0,   3.5, 3.5,-1.5,-1.5,-8.0,-8.0,-5.5,-5.5,-3.5,-3.5])

        # It is needed to determine which task frame is the parent (base)
        # and the name of the new TF
        self.trans.header.frame_id = "odom" # Parent or base frame
        self.trans.child_frame_id = "carrot1"
        # It is needed to stamp (associate a time) to the MTH
        # self.trans.header.stamp = rospy.Time.now()
        
        # # Values for the transformation (self.trans)
        # self.trans.transform.translation.x = trajecx[i]
        # self.trans.transform.translation.y = trajecy[i]
        # self.trans.transform.translation.z = 0.0
        # self.trans.transform.rotation.x = 0.0
        # self.trans.transform.rotation.y = 0.0
        # self.trans.transform.rotation.z = 0.0
        # self.trans.transform.rotation.w = 1.0

        # self.broadcts.sendTransform(self.trans)
        # Polling send trajectory
        self.listn = Listener()
        rate = rospy.Rate(10.0)
        
        
        
        while not rospy.is_shutdown():
                
            # It is needed to stamp (associate a time) to the MTH
            self.trans.header.stamp = rospy.Time.now()
            
            # Values for the transformation (self.trans)
            self.trans.transform.translation.x = trajecx[i]
            self.trans.transform.translation.y = trajecy[i]
            self.trans.transform.translation.z = 0.0
            self.trans.transform.rotation.x = 0.0
            self.trans.transform.rotation.y = 0.0
            self.trans.transform.rotation.z = 0.0
            self.trans.transform.rotation.w = 1.0

            self.broadcts.sendTransform(self.trans)
            a = self.listn.zx()
            print(a)
            
            rate.sleep()

            # Listener
            # if (self.listn.zx()>0):
            # print(self.listn.zx())
            # pause()
            