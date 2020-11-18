#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
# import turtlesim.srv
from kobuki_broadcaster_class import Broadcaster

if __name__ == '__main__':

    rospy.init_node('kobuki_listener')
    rospy.loginfo("Node listener init")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # kobuki_name = rospy.get_param('robot_name', 'kobuki')
    # spawner(4, 2, 0, turtle_name)

    # kobuki_vel = rospy.Publisher('%s/mobile_base/commands/velocity' % kobuki_name, geometry_msgs.msg.Twist, queue_size=1)
    kobuki_vel = rospy.Publisher('/mobile_base/commands/velocity' , geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_footprint', 'carrot1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 1.9 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.05 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        kobuki_vel.publish(msg)

        rate.sleep()