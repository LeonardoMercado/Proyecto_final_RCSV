#!/usr/bin/python

import rospy
import cv2
import numpy as np
from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
azul_min = np.array([110,100,20],np.uint8)
azul_max = np.array([130,255,255],np.uint8)


class image_receive:

    def __init__(self):
        #--- Suscriptor del topico de la camara
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kobuki/camara_principal/image_raw",Image,self.callback)
        #--- Publicador para transmitir las coodenadas x,y de la esfera con respecto a la camara
        self.puntos = Point()
        self.esfera_publicador_posicion  = rospy.Publisher("/esfera_posicion/puntos",Point,queue_size=1)

    def callback(self,data):  #--- Callback del suscriptor
    
        #--- Lectura del grame y conversion usando bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV) # Se convierte al espacio HSV
        mascara_1 = cv2.inRange(img_hsv,azul_min,azul_max)
        mascara_color = cv2.bitwise_and(cv_image,cv_image, mask=mascara_1)
        contornos,_ = cv2.findContours(mascara_1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       

        for c in contornos:
            area = cv2.contourArea(c)
            if area>5000:
                M = cv2.moments(c)
                if (M["m00"]==0): M["m00"]=1
                x = int(M["m10"]/M["m00"])
                y = int(M['m01']/M['m00'])
                cv2.circle(cv_image,(x,y),7,(0,0,255),2)
                fuente = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, 'centro en [{};{}]'.format(x,y),(50,50), fuente, 0.75,(0,255,0),2,cv2.LINE_AA)
                cv2.drawContours(cv_image, [c],0,(255,0,0),3)
                self.puntos.x = x
                self.puntos.y = y
                self.puntos.z = 0
                self.esfera_publicador_posicion.publish(self.puntos)
                #print("El contorno {} tiene un area de {}".format(c,area))

        cv2.line(cv_image, (639,342), (739,342), (0,0,255), 2)
        cv2.line(cv_image, (639,342), (639,442), (0,255,0), 2)
        cv2.imshow('camara',cv_image)
        #cv2.imshow('En HSV',img_hsv)
        #cv2.imshow('Filtrado HSV',mascara_1)
        #cv2.imshow('Filtrado',mascara_color)


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