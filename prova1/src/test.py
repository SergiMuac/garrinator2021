

import math, time

#origen
x0=0#140  #545 mm + (costat/2) del robot
y0=0#140 # (costat/2) del robotv=0.5*3 #volem que el robot es mogui a 0.5m/s
pos=0

while True:
    #~~~~~~~~~~~~~~   Movement block  ~~~~~~~~~~~~~~~~~
    
    x1=int(input('(x) -> '))
    y1=int(input('(y) -> '))

    v = 0.5

    dx = x1 - x0
    dy = y1 - y0

    d=(math.sqrt(dx*dx+dy*dy))/1000    # mm!!
    t=round((d/v)*3,2)      # multipliquem per 3 perque tenim una reduccio de 1/3

    if dx == 0:
        vx=0
        if dy>0:
            alpha=0
            vy=0.5
        elif dy<0:
            alpha=math.pi
            vy=-0.5
        else:
            vy=0
            t=0

    elif dy==0:
        vy=0
        if dx>0:
            alpha=math.pi/2
            vx=0.5
        elif dx<0:
            alpha=3*math.pi/2
            vx=-0.5
        else:
            vx=0
            t=0
    else:
        alpha=math.atan(dx/dy)
        vx=v*math.sin(alpha)
        vy=v*math.cos(alpha)

    print('alpha',alpha)
    print('vx',vx)
    print('vy',vy)
    print('t',t)
    print('d',d)

    x0=x1
    y0=y1
