#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 15:28:21 2023

@author: bekdouche
"""
import numpy as np
import matplotlib.pyplot as plt
from math import *
import random as rd

def normalizeAngle(angle):
    """
    Normalizes an angle to the range [-π, π].

    Parameters
    ----------
    angle : float
        Angle in radians.

    Returns
    -------
    float
        Normalized angle within [-π, π].
    """
    while angle >= np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle 

def drawRobot(x, y, t, color):
    """
    Draws a triangular representation of a robot at a given pose.

    Parameters
    ----------
    x : float
        X-coordinate of the robot.
    y : float
        Y-coordinate of the robot.
    t : float
        Orientation (theta) in radians.
    color : str
        Color used to draw the robot.
    """
    l=0.5 #robot size
    point1 = [x + l*sin(t), y - l*cos(t)]
    point2 = [x + 4*l*cos(t), y + 4*l*sin(t)]
    point3 = [x - l*sin(t), y + l*cos(t)]
    x_values = [point1[0], point2[0], point3[0], point1[0]]
    y_values = [point1[1], point2[1], point3[1], point1[1]]
    plt.plot(x_values, y_values, color)

def calculateAndDrawRobotPose(i, v, w, xPrev, yPrev, thPrev, col):
    """
    Computes the robot's new pose using unicycle model and plots it.

    Parameters
    ----------
    i : int
        Time step index.
    v : float
        Linear velocity.
    w : float
        Angular velocity.
    xPrev, yPrev, thPrev : float
        Previous pose (x, y, theta).
    col : str
        Color for plotting.

    Returns
    -------
    tuple
        New pose (x, y, theta).
    """      
    dT = 0.05 #seconds
    #TODO changer ici
    xCur = xPrev+v*cos(normalizeAngle(thPrev))*dT
    yCur = yPrev+v*sin(normalizeAngle(thPrev))*dT
    thCur = normalizeAngle(thPrev+w*dT)
    #laisser les lignes ci-dessous
    if i % 50 == 0 :
        drawRobot(xCur, yCur, thCur, col)
    plt.plot(xCur, yCur, marker='o', markersize=1, color=col)
    time = i*dT
    print("i is ", i, "time is %.2f" %time, " x %.3f" %xCur, " y %.3f" %yCur, "th %.3f" %thCur)    
    return (xCur, yCur, thCur)    

def simuVerify():
    """
    Simulates random robot motion using random velocities and plots the trajectory.
    """
    # initial position
    x[0] = 0
    y[0] = 0
    th[0] = -np.pi
    drawRobot(x[0], y[0], th[0], 'blue')
    #loop
    for i in range (1,N+1):
        #TODO feedback control
        vel[i] = rd.randint(-2,2)
        omega[i] = rd.randint(-2,2)
        x[i], y[i], th[i] = calculateAndDrawRobotPose(i, vel[i], omega[i], x[i-1], y[i-1], th[i-1], 'lightsteelblue')
    drawRobot(x[i], y[i], th[i], 'midnightblue')
    
def desiredPositionController(xd, yd, x, y, th):
    """
    Computes control commands to move the robot toward a desired position.

    Parameters
    ----------
    xd, yd : float
        Desired position.
    x, y, th : float
        Current pose.

    Returns
    -------
    tuple
        Linear and angular velocities (v, omega).
    """
    dist = sqrt((xd-x)**2+(yd-y)**2)
    vel = dist*0.5
    omega = normalizeAngle(atan2((yd-y)/dist,(xd-x)/dist)-th)
    return vel, omega

def control2DesiredPosition(n, posX, posY, posXd, posYd, theta):
    """
    Controls multiple robots to reach their respective desired positions.

    Parameters
    ----------
    n : int
        Number of robots.
    posX, posY : list
        Initial positions.
    posXd, posYd : list
        Desired positions.
    theta : list
        Initial orientations.
    """
    for i in range(n):
        plt.figure()
        plt.title("Desired Position : ("+str(posXd[i])+', '+str(posYd[i])+')')
        #initial pose
        x[0] = posX[i]; y[0] = posY[i]; th[0] = theta[i]
        drawRobot(x[0], y[0], th[0], 'blue')    
        #desired position
        x_des = posXd[i];  y_des = posYd[i]
        for j in range (1,N+1):
            vel[j], omega[j] = desiredPositionController(x_des, y_des, x[j-1], y[j-1], th[j-1])
            x[j], y[j], th[j] = calculateAndDrawRobotPose(j, vel[j], omega[j], x[j-1], y[j-1], th[j-1], 'lightsteelblue')
            # check distance and if its too small stop the loop
            if abs(sqrt((x[j]-x_des)**2+(y[j]-y_des)**2)) <= 0.001:
                break;
        #draw final robot pose
        drawRobot(x[j], y[j], th[j], 'midnightblue')
        plt.plot(x_des, y_des, 'ko')
        plt.show()

def line_draw(a, b, c, xmin, xmax, ymin, ymax):
    """
    Draws a line defined by ax + by + c = 0 within the given plot bounds.

    Parameters
    ----------
    a, b, c : float
        Line coefficients.
    xmin, xmax, ymin, ymax : float
        Plot boundaries.
    """
    if a == 0:
        if b != 0:
            xline = np.array((xmin, xmax))
            yline = (-c/b, -c/b)
        else:
            return
    elif b != 0:
        xline = np.array((xmin, xmax))
        yline = -a/b*xline-c/b
    else:
        yline = np.array((ymin, ymax))
        xline = (-c/a, -c/a)
    plt.plot(xline, yline)
    
def followPathXController(y, th):
    """
    Controller to follow the x-axis (y = 0) path.

    Parameters
    ----------
    y : float
        Current y-position.
    th : float
        Current orientation.

    Returns
    -------
    tuple
        Linear and angular velocities.
    """
    vel = 2
    d = y
    a = normalizeAngle(th)
    Kd = -0.1
    Ka = -0.5
    omega = normalizeAngle(Kd*d+Ka*a)
    return vel, omega

def followPathX(n, posX, posY, theta):
    """
    Makes robots follow the x-axis using a simple controller.

    Parameters
    ----------
    n : int
        Number of robots.
    posX, posY : list
        Initial positions.
    theta : list
        Initial orientations.
    """
    for i in range(n):
        plt.figure()
        #initial pose
        x[0] = posX[i]; y[0] = posY[i]; th[0] = theta[i]
        drawRobot(x[0], y[0], th[0], 'green')
        #control robot
        for j in range (1,N+1):
            vel[j], omega[j] = followPathXController(y[j-1], th[j-1])
            x[j], y[j], th[j] = calculateAndDrawRobotPose(j, vel[j], omega[j], x[j-1], y[j-1], th[j-1], 'springgreen')
        #draw final robot pose
        drawRobot(x[j], y[j], th[j], 'darkgreen')
        line_draw(0, 1, 0, min(x), max(x), min(y), max(y))
        plt.show()
    
def sign(x):
    """
    Returns the sign of a number.

    Parameters
    ----------
    x : float

    Returns
    -------
    int
        1 if x >= 0, -1 otherwise.
    """
    if x>=0:
        return 1
    else:
        return -1
    
def followPathController(x, y, th, a, b, c):
    """
    Controller to follow a general line defined by ax + by + c = 0.

    Parameters
    ----------
    x, y, th : float
        Current pose.
    a, b, c : float
        Line coefficients.

    Returns
    -------
    tuple
        Linear and angular velocities.
    """
    vel = 2
    d = (a*x+b*y+c)/sqrt(a**2+b**2)
    
    if b == 0:
        beta = -sign(a)*np.pi/2
    elif b>0:
        beta = -atan(a/b)
    else:
        beta = normalizeAngle(-atan(a/b)+np.pi)
    
    Kd = -0.1
    Ka = -0.5
    ang = th-beta
    omega = normalizeAngle(Kd*d+Ka*ang)
    return vel, omega

def followPath(n, posX, posY, theta, a, b, c):
    """
    Makes robots follow specified lines using a path-following controller.

    Parameters
    ----------
    n : int
        Number of robots.
    posX, posY : list
        Initial positions.
    theta : list
        Initial orientations.
    a, b, c : list
        Line coefficients for each robot.
    """
    for i in range(n):
        plt.figure()
        #initial pose
        x[0] = posX[i]; y[0] = posY[i]; th[0] = theta[i]
        drawRobot(x[0], y[0], th[0], 'green')
        #control robot
        for j in range (1,N+1):
            vel[j], omega[j] = followPathController(x[j-1], y[j-1], th[j-1], a[i], b[i], c[i])
            x[j], y[j], th[j] = calculateAndDrawRobotPose(j, vel[j], omega[j], x[j-1], y[j-1], th[j-1], 'springgreen')
        #draw final robot pose
        drawRobot(x[j], y[j], th[j], 'darkgreen')
        line_draw(a[i], b[i], c[i], min(x), max(x), min(y), max(y))
        plt.show()

def rotator(thd, th):
    """
    Computes angular velocity to rotate the robot to a desired orientation.

    Parameters
    ----------
    thd : float
        Desired orientation.
    th : float
        Current orientation.

    Returns
    -------
    tuple
        (0, omega) since only rotation is needed.
    """
    dT = 10
    vel = 0
    omega = normalizeAngle(thd-th)/dT
    return vel, omega

def control2DesiredPose(n, posX, posY, theta, posXd, posYd, thetad):
    """
    Controls robots to reach desired positions and orientations.

    Parameters
    ----------
    n : int
        Number of robots.
    posX, posY : list
        Initial positions.
    theta : list
        Initial orientations.
    posXd, posYd : list
        Desired positions.
    thetad : list
        Desired orientations.
    """
    for i in range(n):
        plt.figure()
        plt.title("Desired Pose : ("+str(posXd[i])+', '+str(posYd[i])+', théta = '+str(round(thetad[i],2))+')')
        #initial pose
        x[0] = posX[i]; y[0] = posY[i]; th[0] = theta[i]
        drawRobot(x[0], y[0], th[0], 'blue')    
        #desired position
        x_des = posXd[i];  y_des = posYd[i]; th_des = thetad[i]
        for j in range (1,N+1):
            vel[j], omega[j] = desiredPositionController(x_des, y_des, x[j-1], y[j-1], th[j-1])
            x[j], y[j], th[j] = calculateAndDrawRobotPose(j, vel[j], omega[j], x[j-1], y[j-1], th[j-1], 'lightsteelblue')
            # check distance and if its too small stop the loop
            if abs(sqrt((x[j]-x_des)**2+(y[j]-y_des)**2)) <= 0.001:
                break
        
        #initialiser la rotation
        x[0] = x[j]; y[0] = y[j]; th[0] = th[j]
        for k in range(1,N+1):
            vel[k], omega[k] = rotator(th_des, th[k-1])
            x[k], y[k], th[k] = calculateAndDrawRobotPose(k, vel[k], omega[k], x[k-1], y[k-1], th[k-1], 'lightsteelblue')
            if abs(th[k]-th_des) <= 0.1:
                break
        
        drawRobot(x[k], y[k], th[k], 'midnightblue')
        plt.plot(x_des, y_des, 'ko')
        plt.show()

def controlLyapounov0(xd, yd, thd, x, y, th):
    """
    Lyapunov-based controller (version 0) for pose regulation.
    Note: Angle correction may not work properly.

    Returns
    -------
    tuple
        Linear and angular velocities.
    """
    dist = sqrt((xd-x)**2+(yd-y)**2)
    angle = atan2((yd-y)/dist,(xd-x)/dist)
    kv = dist*(cos(angle-th)+1)/2
    ko = 3000
    vel = -kv*(((x-xd)*cos(th)+(y-yd)*sin(th)))
    omega = -ko*(th-thd)
    return vel, omega

def controlLyapounov1(xd, yd, thd, x, y, th):
    """
    Lyapunov-based controller (version 1) from an unknown thesis.

    Returns
    -------
    tuple
        Linear and angular velocities.
    """
    kvx = 5
    kvy = 5
    vel = -kvx*(x-xd)*cos(th)-kvy*(y-yd)*sin(th)
    omega = sin(th)*kvx*(x-xd)-kvy*(y-yd)*cos(th)
    return vel, omega


def controlLyapounov(xd, yd, thd, x, y, th):
    """
    Lyapunov-based controller (version 2) with working angle correction.

    Returns
    -------
    tuple
        Linear and angular velocities.
    """
    vel = -((x-xd)*cos(th)+(y-yd)*sin(th))
    omega = -(th-thd)*1000
    return vel, omega

def control2DesiredPoseLyapounov(n, posX, posY, theta, posXd, posYd, thetad):
    """
    Controls multiple robots to reach desired poses (position + orientation)
    using a Lyapunov-based controller.

    Parameters
    ----------
    n : int
        Number of robots.
    posX, posY : list
        Initial positions.
    theta : list
        Initial orientations.
    posXd, posYd : list
        Desired positions.
    thetad : list
        Desired orientations.
    """
    for i in range(n):
        plt.figure()
        plt.title("Desired Pose : ("+str(posXd[i])+', '+str(posYd[i])+', théta = '+str(round(thetad[i],2))+')')
        #initial pose
        x[0] = posX[i]; y[0] = posY[i]; th[0] = theta[i]
        drawRobot(x[0], y[0], th[0], 'blue')    
        #desired position
        x_des = posXd[i];  y_des = posYd[i]; th_des = thetad[i]
        for j in range (1,N+1):
            vel[j], omega[j] = controlLyapounov(x_des, y_des, th_des, x[j-1], y[j-1], th[j-1])
            x[j], y[j], th[j] = calculateAndDrawRobotPose(j, vel[j], omega[j], x[j-1], y[j-1], th[j-1], 'lightsteelblue')
            if abs(th[j]-th_des) <= 0.1 and abs(sqrt((x[j]-x_des)**2+(y[j]-y_des)**2)) <= 0.001:
                break;
        drawRobot(x[j], y[j], th[j], 'midnightblue')
        plt.plot(x_des, y_des, 'ko')
        plt.show()
                
if __name__ =='__main__':
    N = 1000 #iterations
    x = np.zeros(N+1)
    y = np.zeros(N+1)
    th = np.zeros(N+1)
    vel = np.zeros(N+1)
    omega = np.zeros(N+1)
    
    #Question2
    # simuVerify()
    
    # #Question3-4-5
    n = 4
    posX = []
    posXd = []
    posY = []
    posYd = []
    theta = []
    thetad = []
    for i in range(n):
        posX.append(rd.randint(-10,10))
        posXd.append(rd.randint(-10,10))
        posY.append(rd.randint(-10,10))
        posYd.append(rd.randint(-10,10))
        theta.append(rd.randint(-10,10)*0.1*np.pi)
        thetad.append(rd.randint(-10,10)*0.1*np.pi)
        
    # control the object to the desired position
    control2DesiredPosition(n, posX, posY, posXd, posYd, theta)
    # #Question6-7
    followPathX(n, posX, posY, theta)
    # Question8-9
    a = []
    b = []
    c = []
    for i in range(n):
        a.append(rd.randint(-100,100)*0.1)
        if a == 0:
            b.append(sign(rd.randint(-1,1))*rd.randint(1,100)*0.1)
        else:
            b.append(rd.randint(-100,100)*0.1)
        c.append(rd.randint(1,100)*0.1)
    followPath(n, posX, posY, theta, a, b, c)
    # Question10-11
    control2DesiredPoseLyapounov(n, posX, posY, theta, posXd, posYd, thetad)