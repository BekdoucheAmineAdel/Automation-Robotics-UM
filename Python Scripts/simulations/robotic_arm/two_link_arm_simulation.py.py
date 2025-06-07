#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  3 22:50:54 2023

@author: bekdouche
"""

import matplotlib.pyplot as plt
import numpy as np
import random as rd
from math import *

a1 = 0.6
a2 = 0.4
a3 = 0.05
# Question 14
"""
x = a1*cos(th1)+a2*cos(th2+th1)
y = a1*sin(th1)+a2*sin(th2+th1)
"""
# Question 2
"""
Non ce systèm n'est pas lineair on voit bien l'apparition des sinus et cosinus
qu'on peut pas séparer des angles théta1  et théta2
"""
# Question 3
def draw_arms(th1, th2, col, show_hand = False):
    """
    Draws the two-link robotic arm based on joint angles.

    Parameters
    ----------
    th1 : float
        Angle of the first joint (shoulder).
    th2 : float
        Angle of the second joint (elbow).
    col : str
        Color for plotting the arm.
    show_hand : bool, optional
        If True, draws a stylized hand at the end-effector.
    """
    c1 = cos(th1)
    s1 = sin(th1)
    c12 = cos(th2+th1)
    s12 = sin(th2+th1)
    x = [0, a1*c1, a1*c1+a2*c12]
    y = [0, a1*s1, a1*s1+a2*s12]
    if show_hand == True:
        x.extend([a1*c1+a2*c12+a3*s12, a1*c1+a2*c12+a3*s12+a3*c12, a1*c1+a2*c12+a3*s12, a1*c1+a2*c12-a3*s12, a1*c1+a2*c12-a3*s12+a3*c12])
        y.extend([a1*s1+a2*s12-a3*c12, a1*s1+a2*s12-a3*c12+a3*s12, a1*s1+a2*s12-a3*c12, a1*s1+a2*s12+a3*c12, a1*s1+a2*s12+a3*c12+a3*s12])
    plt.plot(x,y,col)
    return None

def draw_centre(th1, th2, col):
    """
    Draws the two-link robotic arm based on joint angles.

    Parameters
    ----------
    th1 : float
        Angle of the first joint (shoulder).
    th2 : float
        Angle of the second joint (elbow).
    col : str
        Color for plotting the arm.
    show_hand : bool, optional
        If True, draws a stylized hand at the end-effector.
    """
    x = 0
    y = 0
    plt.plot(x,y, 'ko-')
    x = a1*cos(th1)
    y = a1*sin(th1)
    plt.plot(x,y,'o-', color = col)
    x = a1*cos(th1)+a2*cos(th1+th2)
    y = a1*sin(th1)+a2*sin(th1+th2)
    plt.plot(x,y,'ro-',)
    return None
def draw_robot(i, th1, th2, col1, col2):
    """
    Draws the robot arm and its joints every 10 iterations.

    Parameters
    ----------
    i : int
        Current iteration index.
    th1 : float
        First joint angle.
    th2 : float
        Second joint angle.
    col1 : str
        Color for the arm.
    col2 : str
        Color for the joints.
    """
    if i%10 == 0:
        draw_arms(th1, th2, col1)
        draw_centre(th1, th2, col2)
    return None

#------------------------------------------------------------------------------
# Question 4
def controlleurQ4(th, angular_velocity, dT):
    """
    Simple integrator controller for joint angle update.

    Parameters
    ----------
    th : float
        Current joint angle.
    angular_velocity : float
        Angular velocity.
    dT : float
        Time step.

    Returns
    -------
    float
        Updated joint angle.
    """
    next_th = th + angular_velocity * dT
    return next_th
def déplacementQ4(init_th, angular_velocity, title = 'Forgotten'):
    """
    Simulates joint motion using constant angular velocity.

    Parameters
    ----------
    init_th : tuple
        Initial joint angles (th1, th2).
    angular_velocity : float
        Constant angular velocity for both joints.
    title : str
        Title of the plot.
    """
    dT = 0.01
    N = 100
    th1 = np.zeros(N)
    th2 = np.zeros(N)
    th1[0], th2[0] = init_th
    plt.figure()
    plt.gca().axis('equal')
    plt.title(title)
    draw_robot(0, th1[0], th2[0], 'blue', 'black')
    for i in range(1,N):
        th1[i] = controlleurQ4(th1[i-1], angular_velocity, dT)
        th2[i] = controlleurQ4(th2[i-1], angular_velocity, dT)
        draw_robot(i, th1[i], th2[i], 'lightsteelblue', 'lightsteelblue')
    draw_robot(0, th1[i], th2[i], 'midnightblue', 'black')
    plt.grid()
    plt.show()
    return None
#------------------------------------------------------------------------------
# Question 6
def controlleur(th, J_inv, Vxd, dT):
    """
    Computes the next joint angles using inverse Jacobian control.

    Parameters
    ----------
    th : ndarray
        Current joint angles (2x1).
    J_inv : ndarray
        Inverse of the Jacobian matrix.
    Vxd : ndarray
        Desired end-effector velocity (2x1).
    dT : float
        Time step.

    Returns
    -------
    ndarray
        Updated joint angles (2x1).
    """
    next_th = th + J_inv.dot(Vxd) * dT
    return next_th
def jacobienne(th0, th1):
    """
    Computes the Jacobian matrix of the 2-link planar arm.

    Parameters
    ----------
    th0 : float
        First joint angle.
    th1 : float
        Second joint angle.

    Returns
    -------
    ndarray
        2x2 Jacobian matrix.
    """
    J = np.array([[-a1*np.sin(th0)-a2*np.sin(th0+th1) , -a2*np.sin(th0+th1)],
        [a1*np.cos(th0)+a2*np.cos(th0+th1) , a2*np.cos(th0+th1)]])
    return J
def déplacementQ7(init_th, Vxd, title = 'Forgotten'):
    """
    Simulates motion using inverse Jacobian control with constant velocity.

    Parameters
    ----------
    init_th : tuple
        Initial joint angles.
    Vxd : ndarray
        Desired end-effector velocity.
    title : str
        Title of the plot.
    """
    dT = 0.01
    N = 100
    th = np.zeros((2, N))
    [[th[0][0]], [th[1][0]]] = init_th
    plt.figure()
    plt.gca().axis('equal')
    plt.title(title)
    draw_robot(0, th[0][0], th[1][0], 'blue', 'black')
    for i in range(1,N):
        J = jacobienne(th[0][i-1], th[1][i-1])
        J_inv = np.linalg.inv(J)
        old_th = np.array([[th[0][i-1]], [th[1][i-1]]])
        [[th[0][i]], [th[1][i]]] = controlleur(old_th, J_inv, Vxd, dT)
        draw_robot(i, th[0][i], th[1][i], 'lightsteelblue', 'lightsteelblue')
    draw_robot(0, th[0][i], th[1][i], 'midnightblue', 'black')
    plt.grid()
    plt.show()
    return None
#------------------------------------------------------------------------------
# Question 10
def déplacementQ10(init_th, Vxd, title = 'Forgotten'):
    """
    Simulates motion using damped least squares inverse Jacobian.

    Parameters
    ----------
    init_th : tuple
        Initial joint angles.
    Vxd : ndarray
        Desired end-effector velocity.
    title : str
        Title of the plot.
    """
    dT = 0.01
    N = 100
    th = np.zeros((2, N))
    [[th[0][0]], [th[1][0]]] = init_th
    plt.figure()
    plt.gca().axis('equal')
    plt.title(title)
    draw_robot(0, th[0][0], th[1][0], 'blue', 'black')
    for i in range(1,N):
        J = jacobienne(th[0][i-1], th[1][i-1])
        TJ = np.transpose(J)
        J_inv = np.linalg.inv(TJ.dot(J) + 1e-4*np.identity(2))@TJ
        old_th = np.array([[th[0][i-1]], [th[1][i-1]]])
        [[th[0][i]], [th[1][i]]] = controlleur(old_th, J_inv, Vxd, dT)
        draw_robot(i, th[0][i], th[1][i], 'lightsteelblue', 'lightsteelblue')
    draw_robot(0, th[0][i], th[1][i], 'midnightblue', 'black')
    plt.grid()
    plt.show()
    return None
#------------------------------------------------------------------------------
def getXY(th1, th2):
    """
    Computes the (x, y) position of the end-effector.

    Parameters
    ----------
    th1 : float
        First joint angle.
    th2 : float
        Second joint angle.

    Returns
    -------
    tuple
        (x, y) coordinates of the end-effector.
    """
    c1 = cos(th1)
    s1 = sin(th1)
    c12 = cos(th2+th1)
    s12 = sin(th2+th1)
    x = a1*c1+a2*c12
    y = a1*s1+a2*s12
    return x,y
def getV(xd, yd, x, y):
    """
    Computes desired velocity vector to reach a target point.

    Parameters
    ----------
    xd, yd : float
        Desired position.
    x, y : float
        Current position.

    Returns
    -------
    ndarray
        Desired velocity vector (2x1).
    """
    dT = 0.01
    k = 0.04
    vx = k*(xd-x)/dT
    vy = k*(yd-y)/dT
    return np.array([[vx], [vy]])
def déplacementQ11(init_th, xd, yd, title = 'Forgotten'):
    """
    Simulates motion to reach a target point using damped least squares.

    Parameters
    ----------
    init_th : tuple
        Initial joint angles.
    xd, yd : float
        Desired end-effector position.
    title : str
        Title of the plot.
    """
    dT = 0.01
    N = 100
    th = np.zeros((2, N))
    th[0][0], th[1][0] = init_th
    plt.figure()
    plt.gca().axis('equal')
    plt.title(title)
    plt.plot(xd, yd, 'ko-')
    draw_robot(0, th[0][0], th[1][0], 'blue', 'black')
    for i in range(1,N):
        J = jacobienne(th[0][i-1], th[1][i-1])
        J = np.array(J)
        J_inv = np.linalg.inv(np.transpose(J) @ J + 1e-4*np.identity(2)) @ np.transpose(J)
        old_th = np.array([[th[0][i-1]], [th[1][i-1]]])
        x, y = getXY(th[0][i-1],th[1][i-1])
        Vxd = getV(xd, yd, x, y)
        [[th[0][i]], [th[1][i]]] = controlleur(old_th, J_inv, Vxd, dT)
        draw_robot(i, th[0][i], th[1][i], 'lightsteelblue', 'lightsteelblue')
    draw_robot(0, th[0][i], th[1][i], 'midnightblue', 'black')
    plt.grid()
    plt.show()
    return None

#------------------------------------------------------------------------------
def getVcirculaire(x0, y0, r, omega, x, y):
    """
    Computes velocity vector to follow a circular trajectory.

    Parameters
    ----------
    x0, y0 : float
        Center of the circle.
    r : float
        Radius of the circle.
    omega : float
        Angular velocity.
    x, y : float
        Current end-effector position.

    Returns
    -------
    ndarray
        Desired velocity vector (2x1).
    """
    dT = 0.01
    k = 0.5
    alpha = atan2(y-y0, x-x0)
    vx = k*(x0+r*cos(alpha+omega)-x)/dT
    vy = k*(y0+r*sin(alpha+omega)-y)/dT
    return np.array([[vx], [vy]])
def déplacementQ12(init_th, r, x0, y0, omega, title = 'Forgotten'):
    """
    Simulates circular motion of the end-effector using inverse kinematics.

    Parameters
    ----------
    init_th : tuple
        Initial joint angles.
    r : float
        Radius of the circle.
    x0, y0 : float
        Center of the circle.
    omega : float
        Angular velocity.
    title : str
        Title of the plot.
    """
    dT = 0.01
    N = 300
    th = np.zeros((2, N))
    th[0][0], th[1][0] = init_th
    plt.figure()
    plt.gca().axis('equal')
    plt.title(title)
    draw_robot(0, th[0][0], th[1][0], 'blue', 'black')
    plt.plot(x0, y0, 'go-')
    for i in range(1,N):
        J = jacobienne(th[0][i-1], th[1][i-1])
        J = np.array(J)
        J_inv = np.linalg.inv(np.transpose(J) @ J + 1e-4*np.identity(2)) @ np.transpose(J)
        old_th = np.array([[th[0][i-1]], [th[1][i-1]]])
        x, y = getXY(th[0][i-1],th[1][i-1])
        Vxd = getVcirculaire(x0, y0, r, omega, x, y)
        [[th[0][i]], [th[1][i]]] = controlleur(old_th, J_inv, Vxd, dT)
        draw_robot(i, th[0][i], th[1][i], 'lightsteelblue', 'lightsteelblue')
    draw_robot(0, th[0][i], th[1][i], 'midnightblue', 'black')
    plt.grid()
    plt.show()
    return None

if __name__ == '__main__':
    # Question 4
    n = 4
    for i in range(n):
        init_th = 2*rd.random()*pi, 2*rd.random()*pi 
        angular_velocity = 2*(rd.random()+0.5)*pi/10
        déplacementQ4(init_th, angular_velocity, 'Test de la question 4 #'+str(i))
    # Question 5
    """
    J = [[-(a1*sin(th1)+2*a2*sin(th1+th2)) , -2*a2*sin(th1+th2)],
        [a1*cos(th1)+2*a2*cos(th1+th2) , 2*a2*cos(th1+th2)]]
    """
    # Question 7
    init_th = np.array([[0.2], [0.6]])
    Vxd = np.array([[-0.1], [0]])
    déplacementQ7(init_th, Vxd, 'Test de la question 7')
    # Question 8
    # init_th = np.array([[0.0], [0.0]])
    # Vxd = np.array([-0.1, 0])
    # déplacementQ7(init_th, Vxd, 'Test de la question 8')
    """
    Commentaire
    Dans cette partie on voit bien que la matrice jacobienne à un
    déterminant nulle et donc c'est impossible de l'inverser
    puisque on est dans un état de singularité
    """
    # Question 9
    init_th = np.array([[0.01], [-0.01]])
    Vxd = np.array([[0.0], [-0.1]])
    déplacementQ7(init_th, Vxd, 'Test de la question 9')
    """
    Commentaire
    Le mouvement est possible dans ce cas puisqu'il n ya pas de singularité
    dans ce point la
    """
    # Question 10
    init_th = np.array([[0.0], [0.0]])
    Vxd = np.array([[-0.1], [0]])
    déplacementQ10(init_th, Vxd, 'Test de la question 8 avec le nouveau système de control')
    """
    Aucun movement sauf que maintenant on voit qu'il n ya pas
    de probleme de singularité
    """
    init_th = np.array([[0.01], [-0.01]])
    Vxd = np.array([[-0.5], [0]])
    déplacementQ10(init_th, Vxd, 'Test de la question 9 avec le nouveau système de control')
    """
    Le movement est correct dans ce sense ce qu'on peut ajouter
    pour améliorer le graph de la question 8 c'est de bornée les
    angle entre 0.01 et 6.27
    """
    
    # Question 11
    init_th = np.array([[0.01], [-0.01]])
    xd = 0.7; yd = 0.3
    déplacementQ11(init_th, xd, yd, 'Déplacement vers la position ('+str(xd)+', '+str(yd)+')')
    
    # Question 12
    init_th = np.array([[0.01], [-0.01]])
    déplacementQ12(init_th, 0.1, 0.5, 0.3, pi/64, 'Simulation de la commande circulaire')