#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct  7 23:52:02 2023
Simulation et commande d’un moteur à courant continu (MCC) et d’un système de positionnement.
@author: Bekdouche Amine
"""

import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *


# =============================================================================
# Définition du système MCC
# =============================================================================

def MCC(C):  
    """
    Crée un modèle espace d'état du moteur à courant continu (MCC).

    Paramètres
    ----------
    C : list
        Vecteur de sélection de la sortie (ex: [0,0,1] pour l'angle).

    Retourne
    -------
    sys : StateSpace
        Représentation espace d'état du MCC.
    """

    # param du motor
    # Paramètres physiques du moteur
    R = 1               # Résistance (Ohm)
    L = 4e-3            # Inductance (H)
    Ke = 0.1            # Constante de force contre-électromotrice
    Km = 0.1            # Constante de couple
    fv = 3e-3           # Coefficient de frottement visqueux
    j = 2e-3            # Inertie (kg.m^2)
    # matrices d'eta
    A = [[-R/L,-Ke/L,0],[Km/j,-fv/j,0],[0,1,0]]
    B = [[1/L],[0],[0]]
    D = [0]
    # equation d'eta
    return ss(A,B,C,D)


# =============================================================================
# Fonction de visualisation position, vitesse, accélération
# =============================================================================

def dessinePosVelAcc(A,B,C,D):  
    """
    Affiche les courbes de position, vitesse et accélération d’un système.

    Paramètres
    ----------
    A, B, C, D : matrices
        Matrices de l’espace d’état du système.
    """

    voitP = ss2tf(A,B,C,D)
    C = np.array([0, 1])
    voitV = ss2tf(A,B, C,D)
    pos ,T = step(voitP)
    vel ,T = step(voitV)
    acc ,T= impulse (voitV)
    plt.figure()
    plt.plot(T, pos)
    plt.plot(T, vel)
    plt.plot(T, acc)
    plt.legend(['pos(t)','vel(t)','acc(t)'])
    plt.xlabel("temps")
    plt.show()


# =============================================================================
# Programme principal
# =============================================================================

if __name__ == '__main__':
    
    # Question 1 & 2 : Création du système MCC
    C = [0,0,1]
    D = [0]
    sys = MCC(C)
    
    # Question 3 : Réponse indicielle
    o, t = step(sys,linspace(0, 0.1,1000))
    plt.figure()
    plt.plot(t,o)
    plt.title('Réponse indicielle (Angle)')
    plt.ylabel('Angle(rad)')
    plt.xlabel('temps (s)')
    plt.grid()
    
    # Question 4 : : Analyse de stabilité
    tf = ss2tf(sys)
    ps = pole(tf)
    """
        -244.86365371+0.j pole stable
        -6.63634629+0.j pole stable
        0.        +0.j pole instable
        
        et donc le system est instable
    """
    # Question 5 : Vérification de la commandabilité
    com = ctrb(sys.A, sys.B)
    """
         [ 2.50000e+02 -6.25000e+04  1.53125e+07]
     det [ 0.00000e+00  1.25000e+04 -3.14375e+06] = 39062499999.99998
         [ 0.00000e+00  0.00000e+00  1.25000e+04]
    le rang de la matrice est égale au nombre d'element du vecteur d'état
    3 = 3
    """
    # Question 6 : Placement de pôles
    p = [-10, -20, -100]
    K = place(sys.A, sys.B, p)
    """
        K = [-0.486  ,  0.14058,  1.6]
    """
    # Question 7 : Système en boucle fermée
    A2 = sys.A-sys.B@K
    B2 = sys.B
    C2 = sys.C
    D2 = sys.D
    sys2 = ss(A2,B2,C2,D2)
    o2, t2 = step(sys2,linspace(0, 1,1000))
    plt.figure()
    plt.plot(t2,o2)
    plt.title('Réponse indicielle (Angle)')
    plt.ylabel('Angle(rad)')
    plt.xlabel('temps (s)')
    plt.grid()
    # Question 8 : Système d’ordre supérieur
    A3 = [[-128.5  ,  -60.145, -400.,   1],
          [  50.   ,   -1.5  ,    0.,   1],
          [   0.   ,    1.   ,    0.,   1],
          [   0.   ,    0.   ,   -1.,   0]]
    B3 = [[0],[0],[0],[1]]
    C3 = [0, 0, 1, 0]
    D3 = D2
    
    sys3 = ss(A3, B3, C3, D3)
    o3, t3 = step(sys3,linspace(0, 40,1000))
    plt.figure()
    plt.plot(t3,o3)
    plt.title('Réponse indicielle (Angle)')
    plt.ylabel('Angle(rad)')
    plt.xlabel('temps (s)')
    plt.grid()
    # Question 9 : Modèle position-vitesse
    """
        x = [p v]
        dx = [v a]
        
        
        v = [0  1] p + [0] a
        a   [0  0] v   [1] 
        
        y = [1 0] x + [0] a
    """
    A4 = [[0,1],[0,0]]
    B4 = [[0],[1]]
    C4 = [1,0]
    D4 = [0]
    sys4 = ss(A4,B4,C4,D4)
    # Question 10 : Fonction de transfert
    tf4 = ss2tf(sys4)
    ps4 = poles(tf4)
    # Question 11 : Réponse indicielle
    o4, t4 = step(tf4,linspace(0, 40,1000))
    plt.figure()
    plt.plot(t4,o4)
    plt.title('Réponse indicielle Distance')
    plt.ylabel('Distance (m)')
    plt.xlabel('temps (s)')
    plt.grid()
    # Question 12 : Vérification de la commandabilité
    com4 = ctrb(sys4.A, sys4.B)
    det_com4 = np.linalg.det(com4)
    """
            [[0., 1.],
       det  [1., 0.]]  = -1.0
        elle est commandable
        2 = len(x)
    """
    
    # Question 13 : Commande LQR
    Q = np.identity(2)
    R = np.identity(1)
    K, S, P = lqr(A4,B4,Q,R)
    A5 = A4-B4@K
    B5 = B4
    C5 = C4
    D5 = D4
    dessinePosVelAcc(A5,B5,C5,D5)
    
    # Question 14 : Placement de pôles manuel
    K2 = place(A4,B4,[-0.5,-2])
    A6 = A4-B4@K2
    B6 = B4
    C6 = C4
    D6 = D4
    dessinePosVelAcc(A6,B6,C6,D6)
    """
        pour calculer le vecteur des gains
        on commence d'abord par placé des
        poles, on commence petit et avec
        des poles stable ensuite on augmente
        petit a petit sans dépasser les limites
        fixer par le system (exemple Courant maximale)
    """
    
