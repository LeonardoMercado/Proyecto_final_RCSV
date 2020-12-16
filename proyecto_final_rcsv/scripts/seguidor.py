#!/usr/bin/python



import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

kp  = 0.0008

def saturacion(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class seguidor_bola():
    def __init__(self):
        
        self.punto_x         = 0.0
        self.punto_y         = 0.0
        self.tiempo_deteccion = 0.0
        
        self.suscriptor_camara = rospy.Subscriber("/esfera_posicion/puntos", Point, self.actualizar_posicion_bola)
        rospy.loginfo("Ya estamos suscripto a la camara.")
        
        self.publicador_velocidad = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=5)
        rospy.loginfo("Publicador de velocidades creado")
        
        self.mensaje = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
    @property
    def detectada(self): return(time.time() - self.tiempo_deteccion < 2.0)
        
    def actualizar_posicion_bola(self, message):
        self.punto_x = message.x
        self.punto_y = message.y
        self.tiempo_deteccion = time.time()
        rospy.loginfo("Bola detectada en: %.1f  %.1f "%(self.punto_x, self.punto_y))

    def controlar(self):
        
        giro = 1.5
        adelante = 0.0
        
        if self.detectada:            
          if (self.punto_x>639 and self.punto_x<1250):
               giro   = -kp * self.punto_x
               print("El valor de velocidad angular quedo en {}".format(giro))
               giro   = saturacion(giro, -1.5, 1.5)
               rospy.loginfo("comando de giro en %.2f"%giro) 
               adelante = 0.75 
          elif (self.punto_x<639 and self.punto_x>25):
               giro   = kp * self.punto_x
               print("El valor de velocidad angular quedo en {}".format(giro))
               giro   = saturacion(giro, -1.5, 1.5)
               rospy.loginfo("comando de giro en %.2f"%giro) 
               adelante = 0.75
          elif self.punto_x == 639:
               giro   = 0
               print("El valor de velocidad angular quedo en {}".format(giro))
               giro   = saturacion(giro, -1.5, 1.5)
               rospy.loginfo("comando de giro en %.2f"%giro) 
               adelante = 0.75

        return (giro, adelante)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action
            giro, adelante    = self.controlar() 
            
            rospy.loginfo("adelanto quedo en = %3.1f"%(giro))
            
            #-- update the message
            self.mensaje.linear.x  = adelante
            self.mensaje.angular.z = giro
            
            #-- publish it
            self.publicador_velocidad.publish(self.mensaje)

            rate.sleep()        
            
if __name__ == "__main__":

    rospy.init_node('seguidor_bola')
    
    seguidor = seguidor_bola()
    seguidor.run()       