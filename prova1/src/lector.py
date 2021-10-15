#! /usr/bin/env python

import rospy, os, math
from sensor_msgs.msg import JointState

def Mostra(mensaje):
    #os.system("clear")
    lista=list(mensaje.position[:4])
    for i in range(0,4):
        lista[i]=round(lista[i],2)
    print(lista)


    l = 0.3        #300x300mm
    w_rad_=0.03      # 60 mm diameter wheel
    sqrt_2=round(math.sqrt(2),2)
    ik_=[[1/sqrt_2, 1/sqrt_2, l/sqrt_2],
        [-1/sqrt_2, 1/sqrt_2, l/sqrt_2],
        [-1/sqrt_2, -1/sqrt_2, l/sqrt_2],
        [1/sqrt_2, -1/sqrt_2, l/sqrt_2]]


rospy.init_node('lector')

# pub1=rospy.Publisher("/omni4_controller/robot/cmd_vel",Twist,queue_size=1)
# pub2=rospy.Publisher('/angle',String,queue_size=1)

while not rospy.is_shutdown():
    rospy.Subscriber('/joint_states',JointState,Mostra)
    rospy.spin()


