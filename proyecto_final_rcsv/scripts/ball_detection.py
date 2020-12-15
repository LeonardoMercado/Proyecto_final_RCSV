#!/usr/bin/python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

azul_min = np.array([100,100,20],np.uint8)
azul_max = np.array([130,255,255],np.uint8)


class image_receive:

    def __init__(self):
        #--- Suscriptor del topico de la camara
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kobuki/camara_principal/image_raw",Image,self.callback)

    def callback(self,data):  #--- Callback del suscriptor
    
        #--- Lectura del grame y conversion usando bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV) # Se convierte al espacio HSV

        mascara_1 = cv2.inRange(img_hsv,azul_min,azul_max)

        cv2.imshow('camara',cv_image)
        cv2.imshow('En HSV',img_hsv)
        cv2.imshow('Filtrado HSV',mascara_1)


        if cv2.waitKey(1) & 0xFF == ord('r'):
            print('----Reiniciando------')
            cv2.destroyAllWindows()
        

#----Funcion Main
def main():
    #--- Creacion del objeto de la clase anteriormente creada
    ic = image_receive()
    
    #--- Inicializacion del nodo
    rospy.init_node('image_receive', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
     
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main()