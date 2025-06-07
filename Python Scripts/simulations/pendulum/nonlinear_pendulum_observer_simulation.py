#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec 10 22:47:14 2023

@author: bekdouche
"""

import numpy as np
import os
from control.matlab import *
import matplotlib.pyplot as plt  # MATLAB plotting functions
from math import *
import math
from scipy import integrate
import scipy as sc

# Use a breakpoint in the code line below to debug your script.
M = 3.2         # Masse du chariot
m = 0.329       # Masse du pendule
J = 0.072       # Iniertie du pendule
l = 0.44        # Longueur du pendule
g = 9.81
fr = 6.2        # Frottement fluide du chariot
fphi = 0.009      # Frottement fluide du pendule
Kf = 2.6        # Coefficient de force [N/V]
n1 = 14.9       # Capteur de position [V/m]
n2 = -52.27     # Capteur d'angle [V/rd]
n3 = -7.64      # Capteur de vitesse [V/m/s]
n4 = -52.27     # Capteur de vitesse angulaire [V/rd/s]
Ks= Kf*1.;        # Coefficient de friction in N
gs= -44.29


# définition des matrices d'état du système linéarisé
# définition de la matrice A
A1 = np.array([[0., 0., 1., 0.], [0., 0., 0., 1.]])
tmp1 = np.array([[M+m, m*l], [m*l, J+m*l*l]])
tmp2 = np.array([[0., 0., -fr, 0.], [0., m*g*l, 0., -fphi]])
A2 = np.linalg.inv(tmp1)@tmp2
A = np.block([[A1], [A2]])
#print("A vaut\n", A)

# définition de la matrice B
B1 = np.array([[0.], [0.]])
tmp3 = np.array([[Kf], [0]])
B2 = np.linalg.inv(tmp1)@tmp3
B = np.block([[B1], [B2]])


# Définitionde la matrice C
C = np.array([[n1, 0, 0, 0], [0, n2, 0, 0], [0, 0, n3, 0]])

# Définition de la matrice D
D = np.array([[0], [0],[0]])

    
# Définition du temps de simulation 
N = 10000

# définiton de la periode d'échantillonage
Te = 0.001


# définition du vecteur d'état initial de la fonction pendule_MNL x = [x1; x2; x3; x4]
x = np.zeros((4, 1))

# définition des matrices nécessaires pour mémoriser le vecteur d'état x dans la matrice xx
# et la dérivée du vecteur d'état x, nommée dx, dans la matrice dxx

dxx = np.zeros((4, N+1))
xx = np.zeros((4, N))       
yy = np.zeros((3, N))       # pour stokker la sortie
t = np.zeros(N)             # pour stocker le temps 

dxx[0, 0] = x[0, 0]
dxx[1, 0] = x[1, 0]
dxx[2, 0] = x[2, 0]
dxx[3, 0] = x[3, 0]
  
# initialisation du vecteur d'état estimé et de la commande
x_hat = np.zeros((5, 1))
u = 0.
xx_hat = np.zeros((5, N))


##############################################################################
#               A ne pas toucher
##############################################################################
def pendule_MNL(u, i):
    
    r       = x[0, 0]
    phi     = x[1, 0]
    dr      = x[2, 0]
    dphi    = x[3, 0]
    
    A1=np.array([[M+m, m*l*cos(phi)], [m*l*cos(phi), J+m*l*l]])
    
    f1 = -fr*dr + m*l*dphi*dphi*sin(phi) + Kf*u - Ks*np.sign(dr);
    f2 = m*g*l*sin(phi) - fphi*dphi;

    dxx1=np.array([[dr], [dphi]])
    dxx2=np.linalg.inv(A1)@np.array([[f1], [f2]])
    dx=np.block([[dxx1], [dxx2]])
    
    dxx[0, i+1]=dx[0]
    dxx[1, i+1]=dx[1]
    dxx[2, i+1]=dx[2]
    dxx[3, i+1]=dx[3]
    
    x[0, 0] = np.trapz([dxx[0, i], dxx[0, i+1]], dx=Te) + x[0, 0]
    x[1, 0] = np.trapz([dxx[1, i], dxx[1, i+1]], dx=Te) + x[1, 0]
    x[2, 0] = np.trapz([dxx[2, i], dxx[2, i+1]], dx=Te) + x[2, 0]
    x[3, 0] = np.trapz([dxx[3, i], dxx[3, i+1]], dx=Te) + x[3, 0]
    
    y = C@x
    
    return y

##############################################################################
#               A mettre ici votre observateur de Kalamn discret
##############################################################################
# Question 7 : 
def observateur(matrixes, u, y, x_hat):
    Aad, Bad, Cad, Dad, Ld = matrixes
    x_hatp = Aad@x_hat+Bad*u+Ld@(y-Cad@x_hat-Dad*u)
    return x_hatp

##############################################################################
#               My implementations
##############################################################################
# Question 1
def stability_analyser(A):
    print("Checking Stability ...")
    pValues, pVectors = np.linalg.eig(A)
    for value in pValues:
        if value.real >= 0:
            print("The système isn't stable due to have a non négative proper value of ", value)
            return
    print("The système is stable and its proper values are ", pValues)
    return

# Question 2
def commandability_analyser(A, B):
    com = ctrb(A, B)
    elements = len(A)
    rank = np.linalg.matrix_rank(com)
    if rank == elements:
        print("The système is commandable")
        return
    print("The système isn't commandable")
    return

def observability_analyser(A, B):
    obsrv = obsv(A, C)
    elements = len(A)
    rank = np.linalg.matrix_rank(obsrv)
    if rank == elements:
        print("The système is observable")
        return
    print("The système isn't observable")
    return

# Question 3
def ss():
    # définition des matrices d'état du système linéarisé augmenté
    # définition de la matrice A
    A1 = np.array([[0., 0., 1., 0., 0.],
                   [0., 0., 0., 1., 0.]])
    tmp1 = np.array([[M+m, m*l],
                     [m*l, J+m*l*l]])
    tmp2 = np.array([[0., 0., -fr, 0., -1.],
                     [0., m*g*l, 0., -fphi, 0.]])
    A2 = np.linalg.inv(tmp1)@tmp2
    Aa = np.block([[A1],
                   [A2],
                   [0., 0., 0., 0., 0.]])
    # print("A vaut\n", A)

    # définition de la matrice B
    B1 = np.array([[0.], [0.]])
    tmp3 = np.array([[Kf], [0]])
    B2 = np.linalg.inv(tmp1)@tmp3
    Ba = np.block([[B1], [B2], [0]])


    # Définitionde la matrice C
    Ca = np.array([[n1, 0, 0, 0, 0], [0, n2, 0, 0, 0], [0, 0, n3, 0, 0]])

    # Définition de la matrice D
    Da = np.array([[0], [0],[0]])
    
    return Aa, Ba, Ca, Da

# Question 4
def ss_discret(Te):
    # calculation des matrices d'état
    Aa, Ba, Ca, Da = ss()
    # discrétisation des matrices d'état
    Aad, Bad, Cad, Dad, Te = sc.signal.cont2discrete((Aa, Ba, Ca, Da), Te, "zoh");
    
    return Aad, Bad, Cad, Dad

# Question 11
def simulateur(position, angle, vitesseChariot, color1, color2, color3):
    x0 = vitesseChariot
    angplus = math.asin(position/l)
    plt.plot(l*sin(angle+angplus), l*cos(angle+angplus),'o-', color = color2)
    plt.plot(x0, 0,'o-', color = color3)
    x, y = (x0,l*sin(angle+angplus)),(0,l*cos(angle+angplus))
    plt.plot(x,y, color1)
    return None
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    
    
    # Question 1 : Analyse de stabilité
    print("Question 1 :")
    stability_analyser(A)
    
    # Question 2 : Analyse de commandabilité du système linéairisé
    print("Question 2 :")
    commandability_analyser(A, B)
    
    
    # Question 2 : Analyse d'observabilité
    observability_analyser(A, C)
    
    # Question 3 
    # définition de la représentation d'état du système augmenté
    print("Question 3 :")
    
    Aa, Ba, Ca, Da = ss()
    
    print("A augmenté : ")
    print(Aa)
    print("B augmenté : ")
    print(Ba)
    print("C augmenté : ")
    print(Ca)
    print("D augmenté : ")
    print(Da)
      
    
    #Question 4
    # utiliser plutôt la fonction cont2discrete de scipy
    print("Question 4 :")
    Aad, Bad, Cad, Dad = ss_discret(Te)
    
    print("A augmenté discrète : ")
    print(Aad)
    print("B augmenté discrète : ")
    print(Bad)
    print("C augmenté discrète : ")
    print(Cad)
    print("D augmenté discrète : ")
    print(Dad)
    
    
    # Question 5 : Calcul des poles de l'observateur de Kalamn discret
    print("Question 5 :")
    poles = [-20., -19., -18., -17., -21.]
    discrete_polesL = [math.exp(pole*Te) for pole in poles]
    
    print("Les poles discrèts sont :")
    print(discrete_polesL)
    
    
    # Question 6 Calcul du gain de Kalman Ld
    print("Question 6 :")
    L = np.transpose(place(np.transpose(Aad), np.transpose(Cad), discrete_polesL))
    
    print("La matrice de gain de l'observateur est :")
    print(L)
    
      
    
    # Question 8 : # placement des poles du retour d'état et calcul de K
    print("Question 8 :")
    Ad, Bd, Cd, Dd, Te = sc.signal.cont2discrete((A, B, C, D), Te, "zoh");
    poles = [-3., -4., -5., -6.]
    discrete_polesK = [math.exp(pole*Te) for pole in poles]
    K = place(Ad, Bd, discrete_polesK)
    
    print("La matrice de commande :")
    print(K)
    
    # Question 9
    rd = 0.2
    Gr = -44.29
    
    plt.figure(0)
    y = pendule_MNL(u, 0)
    simulateur(y[0]/n1, y[1]/n2, y[2]/n3, 'blue', 'blue','green')
    plt.grid()
    for i in range(0, N):
        
        # Appel du système non linéaire
        y = pendule_MNL(u, i)

        # Question 7 : Appel de l'observateur
        x_hat = observateur((Aad, Bad, Cad, Dad, L), u, y, x_hat)
        
        # Question 9 :  Calcul de la commande
        u = x_hat[4]/Kf - K@x_hat[:4] + Gr*rd
        
        ###################################################
        # Sauvegarde des données
        xx_hat[0, i]=x_hat[0]
        xx_hat[1, i]=x_hat[1]
        xx_hat[2, i]=x_hat[2]
        xx_hat[3, i]=x_hat[3]
        xx_hat[4, i]=x_hat[4]
        
        yy[0, i] = y[0]
        yy[1, i] = y[1]
        yy[2, i] = y[2]
        
        t[i]=Te*i
        
        # Question 11
        if i%100 == 0:
            simulateur(y[0]/n1, y[1]/n2, y[2]/n3, 'lightsteelblue', 'red', 'springgreen')
    simulateur(y[0]/n1, y[1]/n2, y[2]/n3, 'midnightblue', 'black', 'darkgreen')
    plt.show()
    
    plt.figure(1)    
    plt.plot(t, yy[0, :]/n1)
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude(m)')
    plt.legend(['y1(t)'])
    plt.title('position du chariot')
    plt.grid()
    
    plt.figure(2)    
    plt.plot(t, yy[1, :]/n2)
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude(rad)')
    plt.legend(['y2(t)'])
    plt.title('position angulaire du pendule')
    plt.grid()
    
    plt.figure(3)    
    plt.plot(t, yy[2, :]/n3)
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude(m/s)')
    plt.legend(['y3(t)'])
    plt.title('vitesse de déplacement du chariot')
    plt.grid()
    
    # Question 10
    """
    Graphique de position:
        Le chariot se stabilise à la position 0.2, 
        conforme à notre consigne initiale. 
        Cette correspondance confirme le bon fonctionnement de notre commande,
        de l'observateur, et la convergence du système.

    Graphique d'angle:
        Un zoom révèle une position angulaire pratiquement nulle,
        en accord avec nos estimations antérieures.

    Graphique de vitesse du chariot:
        Concernant la vitesse de déplacement du chariot, 
        on observe initialement un mouvement négatif anticipé
        pour déplacer le pendule dans la direction des x positifs.
        Ensuite, une régression vers une vitesse positive intervient
        pour ramener le chariot à la position désirée et éviter de la dépasser.
    """
    
    