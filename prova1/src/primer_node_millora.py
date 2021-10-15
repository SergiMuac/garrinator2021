#! /usr/bin/env python

import rospy, math, time, os
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
from std_msgs.msg import Bool
from std_msgs.msg import Int16


#arduino=serial.Serial("/dev/ttyUSB0",baudrate=9600)




class Move() :
    def __init__(self) :
        self.pub1=rospy.Publisher("/omni4_controller/robot/cmd_vel",Twist,queue_size=1)
        self.pub2=rospy.Publisher('/start_stop',String,queue_size=1)
        self.pubArduino=rospy.Publisher('/brazo_arduino',Int16,queue_size=1)
        self.dic_instrucciones={'bajar':90,'subir':0,'faro':80,'coger':300,'soltar':400,'flag':200}

        rospy.Subscriber('/intruso',Bool,self.stop)
        self.data_on=Twist()
        rospy.init_node('primer_node')
        self.count=0
        rate_hz=100
        self.rate=rospy.Rate(rate_hz)
        self.x0=140#140  #545 mm + (costat/2) del robot
        self.y0=140#140 # (costat/2) del robotv=0.5*3 #volem que el robot es mogui a 0.5m/s
        self.pos=0
        self.intrus = False
        self.ordre = 0
        self.w = -1
        self.temps_rotacio = 3.6  #valor teorico 4.71

        #self.coordenades = [[140,670,0,False,0],[250,140],[140,1860],[800,1860]]
        #                       puerto              faro                banderas 1             banderas 2
        self.coordenades = [[140,670,0,False,0],[250,140,1,False,0],[140,1860,1,False,0],[800,1860,1,False,0]]

        while not rospy.is_shutdown() :
            try:
                """
                self.x1=float(input('(x) -> '))
                self.y1=float(input('(y) -> '))
                self.rot=float(input('(giro? 1/0) -> '))
                """
                if self.ordre >= len(self.coordenades) :
                    self.ordre=0
                    
                self.x1 = float(self.coordenades[self.ordre][0])
                self.y1 = float(self.coordenades[self.ordre][1])
                self.rot= int(self.coordenades[self.ordre][2])

                self.v = 0.5

                self.dx = self.x1 - self.x0
                self.dy = self.y1 - self.y0

                # if self.calcul_limit() :
                #     if self.x1<(140+600) :
                #         self.x = self.dx+600
                #         self.y = self.dy
                #         self.d_limit = (math.sqrt(self.x*self.x+self.y*self.y))/1000                         #cas 1

                #     elif self.x1>(2000-140-600) :
                #         self.x = self.dx-600
                #         self.y = self.
                #         self.d_limit = (math.sqrt(self.x*self.x+self.y*self.y))/1000                          #cas 2
                #     if self.y1<(140+600) :      
                #         self.x = 
                #         self.y = 
                #         self.d_limit = (math.sqrt(self.x*self.x+self.y*self.y))/1000                             #cas 3
                #     elif self.y2>(2000-140-600) :       
                #         self.x = 
                #         self.y = 
                #         self.d_limit = (math.sqrt(self.x*self.x+self.y*self.y))/1000                          #cas 4
                    
                #     #extrems!
                    
                    
            
                #    self.temps_limit = round((self.d_limit/self.v)*3,2)      # multipliquem per 3 perque tenim una reduccio de 1/3


                self.d=(math.sqrt(self.dx*self.dx+self.dy*self.dy))/1000    # mm!!
                self.t=round((self.d/self.v)*3,2)      # multipliquem per 3 perque tenim una reduccio de 1/3
                if self.dx == 0:
                    self.vx=0
                    if self.dy>0:    #derecha
                        self.alpha=math.pi/2
                        self.vy=self.v
                    elif self.dy<0:     #izquierda
                        self.alpha=3*math.pi/2
                        self.vy=-self.v
                    else:
                        self.vy=0
                        self.t=0

                elif self.dy==0:
                    self.vy=0
                    if self.dx>0:    #alante
                        self.alpha=0
                        self.vx=self.v
                    elif self.dx<0:       #atras
                        self.alpha=math.pi
                        self.vx=-self.v
                    else:
                        self.vx=0
                        self.t=0
                else:
                    division=self.dy/self.dx
                    self.alpha=math.atan(division)
                    #~~~~~~~~~~~~~~  1r Cuadrante   ~~~~~~~~~~~~~~~~~~~~~
                    if self.dx>0 and self.dy>0:
                        print("1rQ")
                        self.vx=self.v*math.cos(self.alpha)
                        self.vy=self.v*math.sin(self.alpha)

                    #~~~~~~~~~~~~~~  2n Cuadrante   ~~~~~~~~~~~~~~~~~~~~~
                    elif self.dx>0 and self.dy<0:
                        print("2nQ")
                        self.vx=self.v*math.cos(self.alpha)
                        self.vy=self.v*math.sin(self.alpha)

                    #~~~~~~~~~~~~~~  3r Cuadrante   ~~~~~~~~~~~~~~~~~~~~~
                    elif self.dx<0 and self.dy<0:
                        print("3rQ")
                        self.vx=-self.v*math.cos(self.alpha)
                        self.vy=-self.v*math.sin(self.alpha)

                    #~~~~~~~~~~~~~~  4t Cuadrante   ~~~~~~~~~~~~~~~~~~~~~
                    elif self.dx<0 and self.dy>0:
                        print("4tQ")
                        self.vx=-self.v*math.cos(self.alpha)
                        self.vy=-self.v*math.sin(self.alpha)

        #~~~~~~~~~~~~~~~~~~   Publish    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
                print(str(int(self.alpha*180/math.pi)))
                datos=String()
                datos.data='S'+str(int(self.alpha*180/math.pi))
                self.pub2.publish(datos)

        #~~~~~~~~~~~~~~~~~~   Giro    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                self.gir()  #aplica els girs en cas de ser necessaris

        #~~~~~~~~~~~~~~~~~~   Wait    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                #rospy.sleep(self.t)
                self.count=0
                self.data_on.angular.z=0


                while self.count<self.t :
                    
                    self.data_on.linear.x=self.vx
                    self.data_on.linear.y=self.vy


                    self.pub1.publish(self.data_on)
                    # self.pub2.publish(str(int(self.alpha*180/math.pi)))
                    if self.coordenades[self.ordre][3] :
                        if self.count< self.coordenades[self.ordre][4] :
                            while self.intrus :
                                self.data_on.linear.x = 0
                                self.data_on.linear.y = 0
                                self.pub1.publish(self.data_on)
                                self.pub2.publish(str(int(self.alpha*180/math.pi)))
                        
                    self.count+=0.01
                    self.rate.sleep()
                
                

        
        #~~~~~~~~~~~~~~~~~~   Stop    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                datos.data='P'
                self.pub2.publish(datos)

                self.data_on.linear.x=0
                self.data_on.linear.y=0        
                self.pub1.publish(self.data_on)


        #~~~~~~~~~~~~~~~~~~   Info    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                print('alpha',self.alpha)
                print('vx',self.vx)
                print('vy',self.vy)
                print('dx',self.dx)
                print('dy',self.dy)
                print('t',self.t)
                print('d',self.d)

        #~~~~~~~~~~~~~~~~~~   Restart    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                self.count=0
                self.x0=self.x1
                self.y0=self.y1
                self.ordre+=1
                rospy.sleep(2)

            except:
                pass

    def stop (self,intrus) :
        self.intrus = intrus.data
    
    def calcul_limit (self) : 
        if (self.x1 not in range (140+600,3000-600-140)) or (self.y1 not in range (140+600,2000-600-140)):
            return True
        else :
            return False
    
    def gir(self): # puerto (0)    faro (1)->2 giros     banderas1 (2)->1 giro   banderas1 (3) ->1 giro
        if self.ordre==1:
            self.data_on.linear.x = 0
            self.data_on.linear.y = 0
            self.data_on.angular.z = self.w
            self.pub1.publish(self.data_on)
            while self.count<self.temps_rotacio:
                self.count+=0.01
                self.rate.sleep()
            #publicamos mover arduino
            rospy.sleep(2)
            self.data_on.angular.z = -self.w
            self.pub1.publish(self.data_on)
            while self.count<self.temps_rotacio:
                self.count+=0.01
                self.rate.sleep()

            


        
        

        

"""
                self.x1 = float(self.coordenades[self.ordre][0])
                self.y1 = float(self.coordenades[self.ordre][1])
                self.ordre+=1

                self.v = 0.5*2

                self.dx = self.x1 - self.x0
                self.dy = self.y1 - self.y0
"""
            
                        


if __name__ == '__main__':
    try:
        Move() 
        rospy.spin() 

    except rospy.ROSInterruptException:
        pass

    




# def obstacle (caca) :
#     velocitat = Twist()
#     pub=rospy.Publisher("/omni4_controller/robot/cmd_vel",Twist,queue_size=1)
#     if caca.data == True :
#         pub.publish(velocitat)




# rospy.Subscriber('/intruso',Bool,caca)


#rate_hz=100
#rate=rospy.Rate(rate_hz)
#count=0 

#data_off=Twist()
#origen




    #~~~~~~~~~~~~~~   Movement block  ~~~~~~~~~~~~~~~~~
    





    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    """
    #~~~~~~~~~  Servos block  ~~~~~~~~~~~~
    
    angle=str(input("-> "))
    print(type(angle))
    newpunty_angle="X"+new_angle+"Y"
    arduino.write(new_angle)
    #time.sleep(50)

    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~  1er node done  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    """
