#! /usr/bin/env python

import rospy, math, time
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
from std_msgs.msg import Bool

rospy.init_node('primer_node')
#arduino=serial.Serial("/dev/ttyUSB0",baudrate=9600)





# def obstacle (caca) :
#     velocitat = Twist()
#     pub=rospy.Publisher("/omni4_controller/robot/cmd_vel",Twist,queue_size=1)
#     if caca.data == True :
#         pub.publish(velocitat)





pub1=rospy.Publisher("/omni4_controller/robot/cmd_vel",Twist,queue_size=1)
pub2=rospy.Publisher('/angle',String,queue_size=1)
# rospy.Subscriber('/intruso',Bool,caca)


#rate_hz=100
#rate=rospy.Rate(rate_hz)
#count=0 
data_on=Twist()
#data_off=Twist()
#origen
x0=0#140  #545 mm + (costat/2) del robot
y0=0#140 # (costat/2) del robotv=0.5*3 #volem que el robot es mogui a 0.5m/s
pos=0


while not rospy.is_shutdown():
    #~~~~~~~~~~~~~~   Movement block  ~~~~~~~~~~~~~~~~~
    try:
        x1=int(input('(x) -> '))
        y1=int(input('(y) -> '))

        v = 0.5*2

        dx = x1 - x0
        dy = y1 - y0

        d=(math.sqrt(dx*dx+dy*dy))/1000    # mm!!
        t=round((d/v)*3,2)      # multipliquem per 3 perque tenim una reduccio de 1/3

        if dx == 0:
            vx=0
            if dy>0:
                alpha=0
                vy=v
            elif dy<0:
                alpha=math.pi
                vy=-v
            else:
                vy=0
                t=0

        elif dy==0:
            vy=0
            if dx>0:
                alpha=math.pi/2
                vx=v
            elif dx<0:
                alpha=3*math.pi/2
                vx=-v
            else:
                vx=0
                t=0
        else:
            alpha=math.atan(dx/dy)
            if dx<0:
                vx=-v*math.sin(alpha)
            else:
                vx=v*math.sin(alpha)
            if dy<0:
                vy=-v*math.cos(alpha)
            else:
                vy=v*math.cos(alpha)

        #~~~~~~~~~~~~~~~~~~   Publish    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        data_on.linear.x=vx
        data_on.linear.y=vy

        pub1.publish(data_on)
        pub2.publish(str(int(alpha)))

        #~~~~~~~~~~~~~~~~~~   Wait    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        rospy.sleep(t)
        # while count<t :
        #     #print(count)
        #     count+=0.01
        #     rate.sleep()
        
        #~~~~~~~~~~~~~~~~~~   Stop    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        data_on.linear.x=0
        data_on.linear.y=0
        
        pub1.publish(data_on)


        #~~~~~~~~~~~~~~~~~~   Info    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        print('alpha',alpha)
        print('vx',vx)
        print('vy',vy)
        print('t',t)
        print('d',d)

        #~~~~~~~~~~~~~~~~~~   Restart    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #count=0
        x0=x1
        y0=y1

    except:
        pass




    





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