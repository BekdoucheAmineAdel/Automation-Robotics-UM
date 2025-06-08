#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 15:42:53 2023

@author: bekdouche
"""

import matplotlib as plt
import numpy
import pprint
from math import sin,cos,pi,atan2,acos
#Question 3
q1=numpy.array([90,51,42,90])
q2=numpy.array([0,65,21,45])
q1_rad = q1*numpy.pi/180
q2_rad = q2*numpy.pi/180
cos1 = numpy.cos(q1_rad)
cos2 = numpy.cos(q2_rad)
sin1 = numpy.sin(q1_rad)
sin2 = numpy.sin(q2_rad)
x = [229,226,325,181]
z = [178,0,73,63]
sols = []
a,b = [],[]
for i in range(4):
    a.append([0,cos1[i],cos2[i],1])
for i in range(4):
    a.append([1,sin1[i],-sin2[i],0])   
b.extend(x)
b.extend(z)

print('the matrix A :')
pprint.pprint(a)
print('the matrix B :')
pprint.pprint(b)

sols.append(numpy.linalg.lstsq(a,b,rcond=None))

l0_l4=sols[0][0][0]
l1=sols[0][0][1]
l2=sols[0][0][2]
l3=sols[0][0][3]

print('l0-l4 =',l0_l4)
print('l1 =',l1)
print('l2 =',l2)
print('l3 =',l3)

def MGD(q0,q1,q2):
    """
    Function qui va nous donné la position cartésienne finale du robot
    Parameters
    ----------
    q0 : angle deg
    q1 : angle deg
    q2 : angle deg

    Returns
    -------
    x : position dans l'axe des x
    y : position dans l'axe des y
    z : position dans l'axe des x
    On prend comme référentiel la base du robot
    """
    # Conversion de deg vers radian
    rad0,rad1,rad2 = numpy.deg2rad(q0),\
        numpy.deg2rad(q1),numpy.deg2rad(q2)
    
    # Calcule de la longeur d du bras
    d=l1*cos(rad1)+l2*cos(rad2)+l3
    
    # Calcule des cordonnées
    x=d*sin(rad0)
    y=d*cos(rad0)
    z=l1*sin(rad1)-l2*sin(rad2)+l0_l4
    
    return x, y, z
#Question 7
# data = ((90,90,0),\
#         (90,51,65),\
#         (90,42,21),\
#         (90,90,45))
# print("Vérification du modèle")
# print("**************************")
# Data = []
# for angles in data:
#     position = MGD(angles[0],angles[1],angles[2])
#     Data.append(position)
#     print("(q0,q1,q2) =",angles)
#     print("(x,y,z) =", position)
#     print("**************************")
#Question 10
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
    q0 = atan2(x,y)
    #calcule de la longueur de d
    d =numpy.sqrt(x**2+y**2)
    dl3 = d-l3
    zl0l4 = z-l0_l4
    # q2+q1 c'est q2q1 on la calcule avant de proceder
    q2q1 = acos((dl3**2+zl0l4**2-(l1**2+l2**2))/(2*l1*l2))
    print("q2q1 = ",numpy.rad2deg(q2q1))
    # définition de la fraction pour calculer q1
    num = l2*sin(q2q1)*(dl3)+(l1+l2*cos(q2q1))*(zl0l4)
    den = -l2*sin(q2q1)*(zl0l4)+(l1+l2*cos(q2q1))*(dl3)
    # calcule de q1 et détermination de q2 d'apres q2q2-q1
    q1 = atan2(num,den)
    q2 = q2q1-q1
    #convertir les angles en degrée pour les afficher
    q0,q1,q2 = numpy.rad2deg(q0),\
        numpy.rad2deg(q1),numpy.rad2deg(q2)
    return q0,q1,q2

#Question 11
Data = ((229,0,178),\
        (226,0,0),\
        (325,0,73),\
        (181,0,63))
print("Vérification du modèle")
print("**************************")
for distances in Data:
    print("(x,y,z) =",distances)
    print("(q0,q1,q2) =", MGI(distances[0],
                            distances[1],
                            distances[2]))
    print("**************************")