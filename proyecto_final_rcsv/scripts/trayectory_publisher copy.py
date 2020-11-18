#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math
import numpy as np
from kobuki_broadcaster_class import Broadcaster


if __name__ == '__main__':
    
    rospy.init_node('dynamic_tf2_broadcaster')
    rospy.loginfo("Node trajectory")
    


    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "odom"
    t.child_frame_id = "carrot1"

    i=0

    rate = rospy.Rate(10.0)
    # self.modelo = Broadcaster()


    trajecx = np.array([-3.5,-3.5, 1.5, 1.5, 3.5, 3.5,-2.5,-2.5, 1.5, 1.5,-1.0])
    trajecy = np.array([0,   3.5, 3.5,-1.5,-1.5,-8.0,-8.0,-5.5,-5.5,-3.5,-3.5])
    
    while not rospy.is_shutdown():
        # x = rospy.Time.now().to_sec() * math.pi
        
        
        # if (Broadcaster.Marker_Callback<0.1):
        #     i=i+1

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = trajecx[i]
        t.transform.translation.y = trajecy[i]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
        rate.sleep()

        # broad = Broadcaster()
        # print(broad.Marker_Callback())