#!/usr/bin/python

import rospy

import numpy as np
from   rospy.numpy_msg import numpy_msg
from tf.transformations import rotation_matrix 
from   rospy.numpy_msg import numpy_msg

import tf2_ros
import tf_conversions

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

class Broadcaster_Listener:
    
    def __init__(self):
        
        # Atributos
        # Para realizar un broadcast
        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()

        # Para realizar la escucha "listener"
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribers
        rospy.Subscriber("/pose_trayect", numpy_msg(TransformStamped), self.Marker_Callback, queue_size=10 )
        # rospy.Subscriber("topic", Type, callback)
        
        # Trajectory
        self.i=0
        x = rospy.get_param('trayec_py')
        if x==1 :
            print("Entramos a la trayectoria 1")
            
            self.trajecx = np.array([0.01,-3.5,-3.51, 1.5, 1.51, 3.5, 3.5,-2.52,-2.5, 1.52, 1.5,-1.0])
            self.trajecy = np.array([0.01,0.01, 3.51, 3.5,-1.5,-1.5,-8.0,-8.1,-5.5,-5.1,-3.5,-3.51])
        elif x==2 :
            print("Entramos a la trayectoria 2") 
            self.trajecx = np.array([0.01,2.5,2.5,5.0,5.0,9.0,9.0,3.5,2.5,0.01])
            self.trajecy = np.array([0.01,0.01,-8.0,-8.0,-4.0,-4.0,3.0,3.0,0.01,0.01])
        elif x==3 :
            print("Entramos a la trayectoria 3") 
            self.trajecx = np.array([0.001,0.01,1.5,1.5,4.5,-4.5])
            self.trajecy = np.array([0.01,2.5,2.5,-5.0,-5.0,-5.0])


        self.flagAngular = False
        self.flagLinear = False
        # fallo transformacion
        self.flagTrans = True

        self.errorAngular = 0.225
        self.errorLinear = 0.06
        # Publisher
        self.kobuki_vel = rospy.Publisher('/mobile_base/commands/velocity' , numpy_msg(Twist), queue_size=1)

    #--------------------------------------------------------------------------------------#
    # Callback o interrupcion
    def Marker_Callback(self, marker_info):
        self.flagTrans = True

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # Broadcasting of a MTH from the base to carrot1

        # It is needed to stamp (associate a time) to the MTH
        self.transform.header.stamp = rospy.Time.now()

        # It is needed to determine which task frame is the parent (base)
        # and the name of the new TF
        self.transform.header.frame_id = "odom" # Parent or base frame
        self.transform.child_frame_id = "carrot1"    # Name of new TF, created by this MTH
        
        
        
        # Translation part of MTH
        self.transform.transform.translation.x = self.trajecx[self.i]
        self.transform.transform.translation.y = self.trajecy[self.i]
        # self.transform.transform.translation.x = self.trajecx[1]
        # self.transform.transform.translation.y = self.trajecy[1]
        self.transform.transform.translation.z = 0.0

        # Rotation part of MTH
        des_rotation = np.identity(4)   # Even though we only need the rotation matrix
                                        # to use the library of transformations it is
                                        # needed to define a MTH so we create one with
                                        # translational part equal to zero 
        # angle2rotate = 5*np.math.pi/8
        # des_rotation[0,0] =  np.math.cos(angle2rotate)
        # des_rotation[0,1] = -np.math.sin(angle2rotate)
        # des_rotation[1,0] =  np.math.sin(angle2rotate)
        # des_rotation[1,1] =  np.math.cos(angle2rotate)
        # quat_desired = tf_conversions.transformations.quaternion_from_matrix(des_rotation)

        # self.transform.transform.rotation.x = quat_desired[0]
        # self.transform.transform.rotation.y = quat_desired[1]
        # self.transform.transform.rotation.z = quat_desired[2]
        # self.transform.transform.rotation.w = quat_desired[3]
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0
        # if (MTH_base_carrot[0,3]<0.01):
        self.broadcts.sendTransform(self.transform )

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # MTH from base to carrot1
        
        try:
            trans_base_carrot = self.tfBuffer.lookup_transform("base_footprint", "carrot1", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform base-carrot")
            self.flagTrans = False
            return 

        # Creater quaternion vector
        quat_base_carrot = np.array([trans_base_carrot.transform.rotation.x, \
                                    trans_base_carrot.transform.rotation.y, \
                                    trans_base_carrot.transform.rotation.z, \
                                    trans_base_carrot.transform.rotation.w])

        # MTH with position vector equal to zero
        rt_mat_base_carrot = tf_conversions.transformations.quaternion_matrix(quat_base_carrot)

        # Add position vector to MTH
        MTH_base_carrot = rt_mat_base_carrot.copy()
        MTH_base_carrot[0,3] = trans_base_carrot.transform.translation.x
        MTH_base_carrot[1,3] = trans_base_carrot.transform.translation.y
        MTH_base_carrot[2,3] = trans_base_carrot.transform.translation.z
        
        # Rotation part of the MTH
        Rot_base_carrot = rt_mat_base_carrot.copy()
        # print(MTH_base_carrot)
        # print("")

        

        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # MTH from odom to carrot1

        # Transform obtained from ROS
        try:
            trans_odom_carrot = self.tfBuffer.lookup_transform("odom", "carrot1", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform odom-carrot")
            pass
        quat_from_ROS = np.array([trans_odom_carrot.transform.rotation.x, \
                                    trans_odom_carrot.transform.rotation.y, \
                                    trans_odom_carrot.transform.rotation.z, \
                                    trans_odom_carrot.transform.rotation.w])
        rt_mat_from_ROS = tf_conversions.transformations.quaternion_matrix(quat_from_ROS)
        MTH_from_ROS = rt_mat_from_ROS.copy()
        MTH_from_ROS[0,3] = trans_odom_carrot.transform.translation.x
        MTH_from_ROS[1,3] = trans_odom_carrot.transform.translation.y
        MTH_from_ROS[2,3] = trans_odom_carrot.transform.translation.z
        # Rot part
        Rot_odom_carrot = rt_mat_from_ROS.copy()

        # print("MTH odom - carrot1")
        # print(MTH_from_ROS)
        # print("")

        # MTH_from_twoSteps = np.matmul( MTH_odom_cam, MTH_cam_marker)

        # print("MTH obtained by the multiplication of MTHs")
        # print(MTH_from_twoSteps)
        # print("")

        
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        #--------------------------------------------------------------------------------#
        # Moving Kobuki
        msg = geometry_msgs.msg.Twist()      
        #Error Angular
        Rot_error = Rot_base_carrot-Rot_odom_carrot

        rot_error_trans = np.arctan2(trans_base_carrot.transform.translation.y, trans_base_carrot.transform.translation.x)

        # Angular Kobuki Velocity
        # if (np.all(np.absolute(Rot_error) < self.errorAngular ) ):

        # print(quat_from_ROS)

        if (np.absolute(rot_error_trans) > self.errorAngular and (rot_error_trans < 0)):
            msg.angular.z = -np.pi/3
        elif(np.absolute(rot_error_trans) > self.errorAngular and (rot_error_trans > 0)):
            msg.angular.z = np.pi/3
        elif (np.absolute(rot_error_trans) < self.errorAngular):
            self.flagAngular = True
            msg.angular.z = 0

        # if (self.flagAngular):
        #     # msg.angular.z = 1.9 * np.arctan2(trans_base_carrot.transform.translation.y, trans_base_carrot.transform.translation.x)
        #     msg.angular.z = np.pi/3
        # else:
        #     self.flagLinear = True
        #     msg.angular.z = 0

        # Linear Kobuki Velocity
        transl_error_trans = np.sqrt(trans_base_carrot.transform.translation.x ** 2 + trans_base_carrot.transform.translation.y ** 2)

        # if (np.absolute(MTH_base_carrot)[0,3]<self.errorLinear and np.absolute(MTH_base_carrot[1,3])<self.errorLinear and np.absolute(MTH_base_carrot[2,3])<self.errorLinear):
        if (transl_error_trans > self.errorLinear and self.flagAngular):
            msg.linear.x = 0.3
            # self.flagLinear = False
        elif (transl_error_trans < self.errorLinear and self.flagAngular):
            self.flagLinear = True
            msg.linear.x = 0

        # if (self.flagLinear):
        #     # msg.linear.x = 0.05 * np.sqrt(trans_base_carrot.transform.translation.x ** 2 + trans_base_carrot.transform.translation.y ** 2)
        #     msg.linear.x = 0.3
        # else:
        #     msg.linear.x = 0

        self.kobuki_vel.publish(msg)

        # print("Error between the two")
        # print( MTH_from_ROS - MTH_from_twoSteps )
        # print("")

        print(self.i)
        # print("flag linear")
        # print(self.flagLinear)
        # print("flag angular")
        # print(self.flagAngular)
        # print("Cond if angular")
        # print(np.all(np.absolute(Rot_error) < self.errorAngular))

        # if (self.flagLinear and self.flagAngular):
        # if ((transl_error_trans < self.errorLinear) and (np.absolute(rot_error_trans) < self.errorAngular) and self.flagAngular and self.flagLinear and (trans_base_carrot != 0) and (np.absolute(rot_error_trans) != 0) and self.flagTrans):
        if self.flagLinear: 
            self.flagAngular = False
            self.flagLinear = False
            self.i=self.i+1
            if self.i == 12:
                self.i == 11
                

            