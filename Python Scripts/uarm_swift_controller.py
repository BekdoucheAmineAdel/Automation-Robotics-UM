#!/usr/bin/env python3
# Andrea CHERUBINI 
# Université de Montpellier
# lancer ce script, puis 'run file with default configuration' si demandé


import os
import sys
import time
import numpy
import math as m
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from uarm.wrapper import SwiftAPI

"""
api test: send cmd
"""

#INITIALISATION ROBOT REEL
swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})
print('SwiftAPI')
swift.waiting_ready() # wait the rebot ready
print('waiting_ready')
swift.set_acceleration(0.1, wait=True)
swift.set_speed_factor(.05)
rtn = swift.get_mode() 
print('RTN',  rtn )




#FONCTION POUR ENVOYER COMMANDES ARTICULAIRES AU ROBOT REEL
def sendAnglesToRobot(angle0,angle1,angle2,angle3):
    print("sendAnglesToRobot ", round(numpy.degrees(angle0)), round(numpy.degrees(angle1)), round(numpy.degrees(-angle2)), round(numpy.degrees(angle3)))
    #swift veut les angles en degrées
    a3=swift.set_servo_angle(servo_id=3, angle=numpy.degrees(angle3), wait=True)
    a0=swift.set_servo_angle(servo_id=0, angle=numpy.degrees(angle0), wait=True)
    a2=swift.set_servo_angle(servo_id=2, angle=numpy.degrees(angle2), wait=True)
    a1=swift.set_servo_angle(servo_id=1, angle=numpy.degrees(angle1), wait=True)
    #print(a0,a1,a2,a3)
    #print('POSITION AFTER MOTION', swift.get_position() )
    #print( 'ANGLE AFTER MOTION', swift.get_servo_angle() )

# deplacement d'un point initial i a un point final f
sendAnglesToRobot(1.57,1,0.5,1.57)
time.sleep(2)

# sendAnglesToRobot(0,1.57,0,0)
# time.sleep(5)
# sendAnglesToRobot(1.57,1.57,0,0)
# time.sleep(5)
# swift.send_cmd_async('M2231 V1')#ferme  ventouse

# swift.send_cmd_async('M2231 V0')#ouvre ventouse

# Question 12

l0_l4 = 34.50023681310885
l1 = 143.60244182296884
l2 = 161.72913326616128
l3 = 67.1212262725233

def MGI(x,y,z):
    """
    Function qui va nous donné la angles des bras pour ariver a une distination x0,y0,z0
    Parameters
    ----------
    x : position dans l'axe des x
    y : position dans l'axe des y
    z : position dans l'axe des x

    On prend comme référentiel la base du robot
    
    Returns
    -------
    q0 : angle deg
    q1 : angle deg
    q2 : angle deg
    
    """
    
    # calcule de q0
    q0 = m.atan2(x,-y)
    #calcule de la longueur de d
    d =numpy.sqrt(x**2+y**2)
    dl3 = d-l3
    zl0l4 = z-l0_l4
    # q2+q1 c'est q2q1 on la calcule avant de proceder
    q2q1 = m.acos((dl3**2+zl0l4**2-(l1**2+l2**2))/(2*l1*l2))
    # définition de la fraction pour calculer q1
    num = l2*m.sin(q2q1)*(dl3)+(l1+l2*m.cos(q2q1))*(zl0l4)
    den = -l2*m.sin(q2q1)*(zl0l4)+(l1+l2*m.cos(q2q1))*(dl3)
    # calcule de q1 et détermination de q2 d'apres q2q2-q1
    q1 = m.atan2(num,den)
    q2 = q2q1-q1
    #convertir les angles en degrée pour les afficher
    # q0,q1,q2 = numpy.rad2deg(q0),\
    #     numpy.rad2deg(q1),numpy.rad2deg(q2)
    return q0,q1,q2

# question 13
def robot_animation(positions,sleep_time):
    # pos = [x, y, z, etat de  la venteouse 1 ferme 0 ouvre, q3]
    for pos in positions:
        q0,q1,q2 = MGI(pos[0],pos[1],pos[2])
        print(q0,q1,q2)
        sendAnglesToRobot(q0,q1,q2,numpy.radians(pos[4]))
        time.sleep(sleep_time)
        if pos[3] == 1:
            swift.send_cmd_async('M2231 V1')
        else:
            swift.send_cmd_async('M2231 V0')

#     x y z v q3
Pi = [229,0,178,0,0]
Pm1 = [0,170,50,0,0]
Pm2 = [0,170,31,1,0]
Pm3 = [0,170,50,1,180]
Pm3 = [0,-170,50,0,180]
Pf = [0,-170,35,0,180]

# Question 14
robot_animation([Pi,Pm1,Pm2,Pm3,Pf],0) 


sendAnglesToRobot(1.57,1,0.5,1.57)
time.sleep(5)

#TERMINAISON ROBOT
swift.flush_cmd()
swift.disconnect()
print('THE END')