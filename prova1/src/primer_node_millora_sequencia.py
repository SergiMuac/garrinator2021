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
         

        self.inici = False

        rospy.Subscriber('/intruso',Bool,self.stop)
        rospy.Subscriber('/master',String,self.comprova)

        self.data_on=Twist()
        self.msgArduino=Int16()

        rospy.init_node('primer_node')

        self.count=0
        rate_hz=100
        self.rate=rospy.Rate(rate_hz)

        self.intrus = False

        self.ordre = 0

        self.x0=140.0#140  #545 mm + (costat/2) del robot
        self.y0=140.0#140 # (costat/2) del robotv=0.5*3 #volem que el robot es mogui a 0.5m/s

        self.w = -1
        self.temps_rotacio = 3.8  #valor teorico 4.71

        self.dic_instrucciones={'bajar':140,'subir':0,'faro':100,'coger':300,'soltar':400,'flag':200,'miniarm':500}


        
        while not rospy.is_shutdown() :
            try:
                if self.ordre==1:
                    while not self.inici :
                        pass
                if self.camp == 'B' :                   #es necesario apagar el lidar
                    self.coordenades = [[140,545+140,0,False,0,0.2],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [140,300+140,0,False,0.58,0.2],    #1 aprox1 far  (el tiempo real de desactivacion es 0.58)
                            [280+140,300+140,0,False,0.58,0.2],    #2 aprox2 far 
                            [280+140,-(300+140),1,False,0,0],   #3 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,-(280+140),0,False,0,0.3],   #4 bajar brazo + choque faro
                            [230+140,-(280+140),0,False,0,0.1],   #5 subir brazo + aprox faro
                            [280+140,230+140,-1,False,0,0],   #6 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [140.0,2000.0-140.0,0,False,0,1],   #7 bracito+choque banderas
                            [890-140,2000-140,0,False,0,0.35],   # 8banderas
                            [140,1130+140,0,False,0,0.35],   #9 recollir bracito + sud
                            [140,140+1430,0,False,0,0.11],               #10  davant gots
                            [140+120,140+1430,0,False,0,0.2],                 #11  agafar + retorcedir
                            [140+120,545+140,0,False,0,0.2],                     #12 pos port + deixar (copiat)
                            [140,1130+140,0,False,0,0.2]]                     #13  sud (copiat) 
 
                if self.camp == 'A':
                    self.coordenades = [[140,2000-(545+140),0,False,0,0.4],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [140,2000-(230+140),0,False,0.58,0.2],    #1 aprox1 far
                            [20+140,2000-(230+140),0,False,0.58,0.2],    #2 aprox2 far
                            [230+140,2000-(-(280+140)),-1,False,0,0],   #3 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,2000-(-(280+140)),0,False,0,0.4],   #4 bajar brazo + choque faro
                            [230+140,2000-(-(280+140)),0,False,0,0.1],   #5 subir brazo + aprox faro
                            [3000-(280+140),230+140,3,False,0,0],   #6 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [3000-140,2000-140,0,False,0,1.1],   #7 bracito+choque banderas
                            [3000-(890-140),2000-140,0,False,0,0.3],   # 8banderas
                            [3000-(140),1130+140,0,False,0,0]]   #9 recollir bracito + sud
                            

                #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Brac Arduino   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if self.ordre in [4,5,7,9,11,13]:
                    if self.ordre==4:
                        self.msgArduino.data=self.dic_instrucciones['faro']
                        self.pubArduino.publish(self.msgArduino)
                    if self.ordre==7 or self.ordre==10:
                        self.msgArduino.data=self.dic_instrucciones['miniarm']
                        self.pubArduino.publish(self.msgArduino)
                    if self.ordre==5:
                        self.msgArduino.data=self.dic_instrucciones['subir']
                        self.pubArduino.publish(self.msgArduino)
                    if self.ordre==11:
                        self.msgArduino.data=self.dic_instrucciones['bajar']
                        self.pubArduino.publish(self.msgArduino)
                        rospy.sleep(2)
                        self.msgArduino.data=self.dic_instrucciones['coger']
                        self.pubArduino.publish(self.msgArduino)
                        rospy.sleep(2)
                        self.msgArduino.data=self.dic_instrucciones['subir']
                        self.pubArduino.publish(self.msgArduino)
                    if self.ordre==13:
                        self.msgArduino.data=self.dic_instrucciones['bajar']
                        self.pubArduino.publish(self.msgArduino)
                        rospy.sleep(2)
                        self.msgArduino.data=self.dic_instrucciones['soltar']
                        self.pubArduino.publish(self.msgArduino)
                        rospy.sleep(2)
                        self.msgArduino.data=self.dic_instrucciones['subir']
                        self.pubArduino.publish(self.msgArduino)
                    rospy.sleep(2)
                    
                #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Moure Lineal    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if self.ordre in [0,1,2,4,5,7,8,9,10,11,12,13]: 
                    self.x1 = float(self.coordenades[self.ordre][0])
                    self.y1 = float(self.coordenades[self.ordre][1])
                    self.v = 0.5*2

                    if self.ordre==4:
                        self.v = 0.5*2
                    #[230+140,-(280+140),1,False,0,0]

                    self.dx = self.x1 - self.x0
                    self.dy = self.y1 - self.y0

                    self.calcul_lineal()

                    if self.ordre ==7 and self.camp == 'B':
                        self.alpha=100*math.pi/180

                    datos=String()
                    datos.data='S'+str(int(self.alpha*180/math.pi))
                    self.pub2.publish(datos)

                    while self.count<self.t+float(self.coordenades[self.ordre][5]) :
                        self.data_on.linear.x=self.vx
                        self.data_on.linear.y=self.vy

                        self.pub1.publish(self.data_on)

                        if self.coordenades[self.ordre][3] :            #es necessari apagar el lidar?
                            if self.count<self.coordenades[self.ordre][4] :
                                while self.intrus:
                                    print("intruso")
                                    self.data_on.linear.x=0
                                    self.data_on.linear.y=0
                                    self.pub1.publish(self.data_on)
                        """
                        else: ##no es necessari apagar el lidar
                            while self.intrus:
                                print("intruso")
                                self.data_on.linear.x=0
                                self.data_on.linear.y=0
                                self.pub1.publish(self.data_on)
                                #self.pub2.publish(str(int(self.alpha*180/math.pi)))"""

                            
                        self.count+=0.01
                        self.rate.sleep()


                #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Moure angular    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                if self.ordre in [3,6]:
                    self.x1 = float(self.coordenades[self.ordre][0])
                    self.y1 = float(self.coordenades[self.ordre][1])

                    self.sentit= int(self.coordenades[self.ordre][2])
                    self.gir()  #aplica els girs en cas de ser necessaris


        #~~~~~~~~~~~~~~~~~~   Stop    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                datos.data='P'
                self.pub2.publish(datos)

                self.data_on.angular.z=0
                self.data_on.linear.x = 0
                self.data_on.linear.y = 0

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
                rospy.sleep(1)

            except:
                pass

    def calcul_lineal(self):
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
            print(division)
            self.alpha=math.atan(division)
            print(self.alpha)
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

    def stop (self,intrus) :
        self.intrus = intrus.data
    
    def gir(self):  # tots els girs son de 90 graus aprox
        self.data_on.angular.z = self.w*self.sentit
        self.pub1.publish(self.data_on)
        if self.sentit==3:
            while self.count<self.temps_rotacio-0.2:
                self.count+=0.01
                self.rate.sleep()
        else:
            if self.ordre==6 and self.camp=='B':
                while self.count<self.temps_rotacio-0.35:
                    self.count+=0.01
                    self.rate.sleep()
            else:
                while self.count<self.temps_rotacio:
                    self.count+=0.01
                    self.rate.sleep()

    def comprova (self,lletra) :
        if lletra.data == 'S' :
            self.inici = True
        if lletra.data == 'A' or lletra.data == 'B' :
            self.camp = lletra.data
        if lletra.data == 'F' :
            self.msgArduino.data = self.dic_instrucciones['flag']     
            self.pubArduino.publish(self.msgArduino)

        
        

        

"""
                self.x1 = float(self.coordenades[self.ordre][0])
                self.y1 = float(self.coordenades[self.ordre][1])
                self.ordre+=1

                self.v = 0.5*2

                self.dx = self.x1 - self.x0
                self.dy = self.y1 - self.y0

                v3
                if self.camp == 'B' :                   #es necesario apagar el lidar
                    self.coordenades = [[140,545+140,0,True,0,0.2],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [280+140,230+140,0,True,0.58,0.2],    #1 aprox far
                            [230+140,-(280+140),1,True,0,0],   #2 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,-(280+140),0,True,0,0.3],   #3 bajar brazo + choque faro
                            [230+140,-(280+140),0,False,0,0.1],   #4 subir brazo + aprox faro
                            [280+140,230+140,-1,True,0,0],   #5 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [140,2000-140,0,False,0,0.8],   #6 bracito+choque banderas
                            [1000+140,2000-140,0,False,0,0.3],   # 7banderas
                            [140,1130+140,0,False,0,0]]   #8 recollir bracito + sud
                if self.camp == 'A':
                    self.coordenades = [[140,2000-(545+140),0,True,0,0.4],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [280+140,2000-(230+140),0,True,0.58,0.2],    #1 aprox far
                            [230+140,2000-(-(280+140)),-1,True,0,0],   #2 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,2000-(-(280+140)),0,True,0,0.3],   #3 bajar brazo + choque faro
                            [230+140,2000-(-(280+140)),0,False,0,0.1],   #4 subir brazo + aprox faro
                            [3000-(280+140),230+140,2.95,True,0,0],   #5 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [3000,2000-140,0,False,0,0.8],   #6 bracito+choque banderas
                            [3000-(1000+140),2000-140,0,False,0,0.3],   # 7banderas
                            [3000-(140),1130+140,0,False,0,0]]   #8 recollir bracito + sud

                v2
                self.coordenades = [[140+50,2000-(545+140),0,False,0,0.2],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [280+140,2000-(230+140),0,False,0,0.2],    #1 aprox far
                            [230+140,2000-(-(280+140)),-1,False,0,0],   #2 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,2000-(-(280+140)),0,False,0,0.3],   #3 bajar brazo + choque faro
                            [230+140,2000-(-(280+140)),0,False,0,0.1],   #4 subir brazo + aprox faro
                            [280+140,2000-(230+140),1,False,0,0],   #5 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [140,2000-(2000-140),0,False,0,1.0],   #6 bracito+choque banderas
                            [1000+140,2000-(2000-140),0,False,0,0.3],   # 7banderas
                            [140,2000-(1130+140),0,False,0,0]]   #8 recollir bracito + sud





                v1


                self.coordenades = [[140,545+140,0,False,0,0.2],    #0 posicio port  ~~~ X,Y ~~~ Respecte el camp
                            [280+140,230+140,0,False,0,0.2],    #1 aprox far
                            [230+140,-(280+140),1,False,0,0],   #2 rotacion 90  ~~~ X,Y ~~~ Respecte el camp -> X=Y i Y=-X
                            [140,-(280+140),0,False,0,0.3],   #3 bajar brazo + choque faro
                            [230+140,-(280+140),0,False,0,0.1],   #4 subir brazo + aprox faro
                            [280+140,230+140,-1,False,0,0],   #5 rotacion -90  ~~~ X,Y ~~~ Respecte el camp
                            [280+140,2000-140,0,False,0,0.5],   #6 choque banderas   [280+240,2000-140,0,False,0,0.5]
                            [1000+140,2000-140,0,False,0,0],   # 7bracito+banderas
                            [140,1130+140,0,False,0,0]]   #8 recollir bracito + sud
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
    

                # """
                # self.x1=float(input('(x) -> '))
                # self.y1=float(input('(y) -> '))
                # self.rot=float(input('(giro? 1/0) -> '))
                # """
                

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
