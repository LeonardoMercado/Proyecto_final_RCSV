#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
import os


Datos = "/home/leonardo/Documentos/01_RCSV/05_Proyecto_Final/Proyecto_final_RCSV/proyecto_final_rcsv/caja_1/p"
#os.chdir('/Documentos/01_RCSV/05_Proyecto_Final/Proyecto_final_RCSV/proyecto_final_rcsv/caja_1/p')
"""
if not os.path.exists(Datos):
    print('Carpeta creada: ',Datos)
    os.makedirs(Datos, 777)
"""
x_l_1, y_l_1 = 0, 0
x_l_2, y_l_2 = 570, 720

x_r_1, y_r_1 = 750, 0
x_r_2, y_r_2 = 1280, 720



class image_receive:

    def __init__(self):
        #--- Suscriptor del topico de la camara
        self.count = 205
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kobuki/camara_principal/image_raw",Image,self.callback)

    def callback(self,data):  #--- Callback del suscriptor
    
        #--- Lectura del grame y conversion usando bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        aux_img = cv_image.copy()
        #cv2.rectangle(cv_image,(x_l_1,y_l_1),(x_l_2,y_l_2),(255,0,0),2)
        cv2.rectangle(cv_image,(x_r_1,y_r_1),(x_r_2,y_r_2),(0,0,255),2)
        #objeto = aux_img[y_l_1:y_l_2 , x_l_1:x_l_2]
        objeto = aux_img[y_r_1:y_r_2 , x_r_1:x_r_2]
        objeto = imutils.resize(objeto, width=38)

        k = cv2.waitKey(1)

        if k == ord('s'):             
             cv2.imwrite(os.path.join(Datos,'objeto{}.jpg'.format(self.count)),objeto)
             print('Imagen guardada:'+'/objeto_{}.jpg'.format(self.count))
             self.count = self.count +1
        



        cv2.imshow("Imagen capturada", cv_image)
        cv2.imshow("recorte",objeto)
        cv2.waitKey(3)

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