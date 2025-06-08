#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 13:26:47 2023
Simulation d’un système continu et discret basé sur un moteur à courant continu (MCC).
@author: Bekdouche Amine
"""

import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *

"""
Simulation d'un systeme continue
"""

# Question 3

# =============================================================================
# Définition du système MCC (Moteur à Courant Continu)
# =============================================================================

def MCC(C):
    
    """
    Crée un modèle d'état du moteur à courant continu (MCC).

    Paramètres
    ----------
    C : list
        Vecteur de sélection de la sortie (ex: [1,0,0] pour le courant).

    Retourne
    -------
    sys : StateSpace
        Représentation espace d'état du système MCC.
    """

    # Paramètres physiques du moteur
    R = 1.52        # Résistance (Ohm)
    L = 2.2e-3      # Inductance (H)
    Ke = 0.127      # Constante de force contre-électromotrice
    Km = 0.127      # Constante de couple
    fv = 5.06e-5    # Coefficient de frottement visqueux
    j = 8.3e-5      # Inertie (kg.m^2)
    # matrices d'eta
    A = [[-R/L,-Ke/L,0],[Km/j,-fv/j,0],[0,1,0]]
    B = [[1/L],[0],[0]]
    D = [0]
    # equation d'eta
    return ss(A,B,C,D)

# Question 7

# =============================================================================
# Simulation d'une équation récurrente (discrète)
# =============================================================================

def equ2(N):
    """
    Calcule la réponse d’un système discret défini par une équation récurrente.

    Paramètres
    ----------
    N : int
        Nombre d’échantillons.

    Retourne
    -------
    y : list
        Réponse du système.
    """
    y = [0,0]
    for i in range(2,N):
        temp = 0.1*y[i-1]-0.7*y[i-2]+1
        y.append(temp)
    return y


# =============================================================================
# Programme principal
# =============================================================================
if __name__ == '__main__':
    # Question 1
    """
    vecteur d'etat:
        x(t) = [[i(t)], [w(t)], [o(t)]]
        le choix du vecteur d'état ce fait en analysant les équations
        du systeme
        on trouve que notre sortie est o(t)
        pour répondre au fait qu'on a la dérivé second de o(t), dw,
        on doit ajouter le w(t) dans le vecteur aussi
        et puisque o(t) c'est le parametre de sortie on trouve
        x(t)=[i(t), w(t), o(t)]
    """
    # Question 2
    """
        dX = A X + B U
        Y = C X + D U
        ------ 
        | di |    |-R/L -Ke/L 0|   | i |   |1/L|
        | dw | =  |Km/j -fv/j 0| * | w | + |0  | * u
        | do |    |0    1     0|   | o |   |0  |
        -----
                           | i |   
        | y | =  |x x x| * | w | + |0| * u
                           | o |   
        -----
    """
    # Question 4 : Réponses indicielle du système MCC
    sys1 = MCC([1,0,0])
    sys2 = MCC([0,1,0])
    sys3 = MCC([0,0,1])
    
    
    i,t = step(sys1,linspace(0, 0.1,1000))
    w,t = step(sys2,linspace(0, 0.1,1000))
    o,t = step(sys3,linspace(0, 0.1,1000))
    
    # Tracé des réponses
    plt.figure()
    plt.plot(t,i)
    plt.title('Réponse indicielle (Courant)')
    plt.ylabel('Courant(A)')
    plt.xlabel('temps (s)')
    plt.grid()
    """
    dans un premier temps on a besoin du courant pour augmenter la vitesse de rotation
    apres avoir arrivé a une valeur précise le courant n'est plus nécessaire sauf pour eliminer l'effet
    du frottement visqueux.
    """
    plt.figure()
    plt.plot(t,w)
    plt.title('Réponse indicielle (Vitesse)')
    plt.ylabel('Vitesse(rad/s)')
    plt.xlabel('temps (s)')
    plt.grid()
    """
    la vitesse converge vers une valeur constant parce que cette entre est stabilisé par le system 
    """
    plt.figure()
    plt.plot(t,o)
    plt.title('Réponse indicielle (Angle)')
    plt.ylabel('Angle(rad)')
    plt.xlabel('temps (s)')
    plt.grid()
    """
    on sait que le system est instable ce qui explique la divergence du system infiniment
    la linearity de la divergence est liée au fait que la vitesse converge vers une valeur constante.
    """
    
    # Question 5 : Fonction de transfert de la vitesse
    tf_w = ss2tf(sys2)
    
    """
            6.955e+05
    ------------------------- => 2éme ordre
    s^2 + 691.5 s + 8.875e+04
    """
    
    """
    Simulation des systèmes numériques
    """
    
    # Question 6
    """
        Pour calculer y(k+2) on a besoin de y(k+1), y(k), et u(k)
        il suffit de connaitre l'entree u(k), et les deux valeurs initial de y(k), (y(0),y(1))
        avec ces grandeur on peut trouver toutes les futures valeurs de y(k)
    """
    
    # Question 8 : Simulation numérique de l’équation récurrente
    n = 100
    yk = equ2(n)
    k = range(n)
    xk = np.ones((n,1))
    plt.figure()
    plt.plot(xk)
    plt.plot(yk)
    plt.title('tracé de yk,xk')
    plt.legend(['y','x'])
    plt.xlabel('amplitude')
    plt.xlabel('k')
    plt.grid()
    """
        On remarque que la réponse indicielle converge et donc on a un système stable
        avec des oscillations puisque le système est du deuxieme degrée
    """
    
    # Question 9 : Fonction de transfert discrète
    num4 = [1]
    denum4 = [1,-0.1,0.7]
    tf4 = tf(num4,denum4,1)
    yk2, k = step(tf4,linspace(0, 99,100))
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(yk)
    plt.title('tracé de la réponse indicielle')
    plt.ylabel('y')
    plt.grid()
    plt.subplot(2,1,2)
    plt.plot(yk2)
    plt.title('tracé de la réponse indicielle TF')
    plt.xlabel('k')
    plt.grid()
    plt.tight_layout()
    """
        On remarque que la réponse indicielle est identique à celle de la
        réponse précédente (équation récurrente)
    """
    
    
    # Question 10 : Modèle espace d’état discret
    Ad = [[0,1],[-0.7,0.1]]
    Bd = [[0],[1]]
    Cd = [1,0]
    Dd = [0]
    eq_etat_num = ss(Ad,Bd,Cd,Dd,1)
    yk3, k3 = step(eq_etat_num, linspace(0, 99,100))
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(yk)
    plt.title('tracé de la réponse indicielle')
    plt.grid()
    plt.subplot(3,1,2)
    plt.plot(yk2)
    plt.title('tracé de la réponse indicielle TF')
    plt.ylabel('y')
    plt.grid()
    plt.subplot(3,1,3)
    plt.plot(yk3)
    plt.title('tracé de la réponse indicielle EE')
    plt.xlabel('k')
    plt.grid()
    plt.tight_layout()
    """
        On remarque que la réponse indicielle est identique au deux
        réponses précédentes (équation récurrente, fonction de transfert)
    """
    
    # Question 11 : Conversion EE -> TF
    tf4_ss = ss2tf(eq_etat_num)
    """
         4.163e-17 z + 1
        -----------------
        z^2 - 0.1 z + 0.7
        
        puisque 4.163e-17 ≈ 0
        
               1
       -----------------
       z^2 - 0.1 z + 0.7
       
       qui est égale a (4)
    """
    # Question 12 : Conversion TF -> EE
    ss_tf4 = tf2ss(tf4)
    """
            ￼￼￼ 1
        ----------------- ........ (4)
        z^2 - 0.1 z + 0.7
        
        A = [[ 0.1 -0.7]
             [ 1.   0. ]]
        B = [[1.]
             [0.]]
        C = [[0. 1.]]
        D = [[0.]]
        dt = 1
        
        Une différence entre R1 et R2 existe et elle due au changement du
        vecteur d'état
        X(R1) = [y(k), y(k+1)]
        X(R2) = [y(k+1), y(k)]
    """
