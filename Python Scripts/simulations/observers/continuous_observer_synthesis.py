"""
Created on Thu Nov 19 11:02:46 2020
synthèse d'observateur pour des systèmes linéaiers continus
@author: Bekdouche Amine
"""


# TP 3 Observabilité et synthèse d'observateur
# partie 2 : synthèse d'observateur pour des systèmes linéaiers continus


import numpy as np
import os
from control.matlab import *
import matplotlib.pyplot as plt  # MATLAB plotting functions
import sympy as sp

# Fonction systeme
def systeme():
    
    A = [[0,0,1], [1,-1,2], [-6,0,-5]]
    B = [[1], [2], [1]]
    C = [1,1,1]
    D = [0]
    
    return ss(A,B,C,D)


# fonctin qui simule un système continu (à ne pas toucher)
# Created by Salhi Abdelaziz
def system(t, u):
    # Définition des matrices d'état
    A = [[0., 0., 1.], [1., -1., 2.],[-6., 0., -5.]]
    B = [[1.], [2.], [1.]]
    C = [1., 1., 1.]
    D = [0.]
    # vecteur d'état initial
    x0 = [[4.0], [-3.0], [1.0]]

    # Calcul des valeurs propres et de la matrice de passage
    valPropres, T = np.linalg.eig(A)

    # les valeurs propres de A
    lambda1 = valPropres[0]
    lambda2 = valPropres[1]
    lambda3 = valPropres[2]
    
     # La matrice diagonalisée de A
    D = [[lambda1, 0., 0.], [0., lambda2, 0.],[0., 0., lambda3]]
    
    # Calcul de la matrice exponentiel e(At)
    exp_Dt = [[np.exp(lambda1*t), 0., 0.], [0., np.exp(lambda2*t), 0.], [0., 0., np.exp(lambda3*t)]]
    
    # Ce calcul est expliqué dans le cours (slide 30) 
    exp_At = T@exp_Dt@np.linalg.inv(T)
    
    ## calcul de la solution de l'équation différentielle (slide 26)
    
    # calcul de la solution homogène (slide 25)
    xh = exp_At @ x0
    
    # calcul de la solution particulière (slide 26)
    xp = -np.linalg.inv(A)@B + exp_At @ np.linalg.inv(A) @ B 
    
    # solution complète de l'équation détat
    x = xh + xp
    
    # calcul de la sortie du système  
    y = C @ x
    
    return y

##############################################################################
# La fonction observateur à mettre ici 
    
def observateur(u, y, xhat):
    Phi = np.array([[ 9.99997005e-01,  0.00000000e+00,  9.97503164e-04],
                    [ 9.93511156e-04,  9.99000500e-01,  1.99450733e-03],
                    [-5.98501898e-03,  0.00000000e+00,  9.95009489e-01]])
    
    Gamma = np.array([[0.0010005 ], [0.0020005 ], [0.00099451]])
    
    LTe = np.array([[[-0.006],  [0.009],  [0.006]]])
    
    C = np.array([1., 1., 1.])
    
    xhatp = Phi@xhat+Gamma*u+LTe*(y-C@xhat)
    
    return xhatp
    
##############################################################################

if __name__ == '__main__':  
    # Question 1
    """
        Fonction Systeme
    """
    # Question 2
    sys1 = systeme()
    y1, t1 = step(sys1,linspace(0, 15,1000))
    
    plt.figure()
    plt.plot(t1, y1)
    plt.title('Réponse indicielle')
    plt.ylabel('Amplitude')
    plt.xlabel('temps (s)')
    plt.grid()
    """
    Ce système présente un réponse a un echlon
    on voit qu'il exist un dépassement et donc le poles
    sont imaginaires
    """
    # Question 3
    mat_obv1 = np.vstack([sys1.C, sys1.C@sys1.A, sys1.C@sys1.A**2])
    det_mat1 = np.linalg.det(mat_obv1)
    print("\n Question 3\n")
    print("Matrice d'observabilité (calcule manuel) :\n", mat_obv1)
    print("Déterminant de cette matrice :\n", det_mat1)
    mat_obv2 = obsv(sys1.A, sys1.C)
    det_mat2 = np.linalg.det(mat_obv2)
    print("Matrice d'observabilité (function obsv) :\n", mat_obv2)
    print("Déterminant de cette matrice :\n", det_mat2)
    """
    Le résultat de la function obsv est identique a celui de qu'on vien de
    calculer
    Le système est observable puisque le rang de la matrice d'observabilité
    est non nul
    """
    # Question 4
    """
        dx1 = A x1 + B u
        y1  = C x1
        
        dx2 = A x2 + B u  + L (y1 - y2)
        y2  = C x2
        
        e   =  x1 - x2
        de  =  dx1 - dx2
        
        de = A x1 + B u - A x2 - B u  - L (y1 - y2)
        de = A x1 + B u - A x2 - B u  - L C x1 + L C x2
        de = A e - L C e
        de = (A - LC) e
    """
    # Question 5
    """
        det(s * I - A - LC) = 0
                      | 0-L1  0-L1   1-L1|
        A' = A - LC = | 1-L2 -1-L2   2-L2|
                      |-6-L3  0-L3  -5-L3|
                      
                      
                      |s+L1       L1     L1-1|
        M = sI - A' = |L2-1   s+1+L2     L2-2|
                      |L3+6       L3   s+L3+5|
                      
        det(M) = L1*s**2 + L1*s - 8*L1 + L2*s**2 + 5*L2*s + 6*L2 + L3*s**2 + 4*L3*s + 2*L3 + s**3 + 6*s**2 + 11*s + 6
        
        15 = L1 +  + L2  + L3   +6 
        74 = L1 + 5*L2+ 4*L3+ 11
        120 = - 8*L1 + 2*L3 + 6 + 6*L2
        
        {L1: -6, L2: 9, L3: 6}
    """
    s, L1, L2, L3 = sp.symbols("s L1 L2 L3")

    M = sp.matrices.Matrix(((s+L1,       L1,     L1-1) ,(L2-1,   s+1+L2,     L2-2),(L3+6,       L3,   s+L3+5)))

    detM = M.det()

    f3 = L1 +  + L2  + L3   +6 
    f2 = L1 + 5*L2+ 4*L3+ 11
    f1 = - 8*L1 + 2*L3 + 6 + 6*L2

    eq1 = sp.Eq(f3,15)
    eq2 = sp.Eq(f2,74)
    eq3 = sp.Eq(f1,120)

    solution = sp.solve((eq1,eq2,eq3),(L1, L2, L3))
    
    print("\n Question 5\n")
    
    print("the solution is:", solution)
    
    
    L = place(sys1.A.T, sys1.C.T, (-4, -5, -6))
    print("La matrice L :\n", L)
    # Question 6
    Te = 1e-3
    sys1_d = c2d(sys1, Te)
    
    Phi = sys1_d.A
    Gamma = sys1_d.B
    
    print("Phi :\n", Phi)
    print("Gamma :\n", Gamma)
    # Question 7
    """
         Fonction Observateur
    """
    # Question 8
    #############################################################
    # On simule le système pendant 10 s (= N*Te) 
    # N est le nombre d'échantillon
    N = 10000
    
    # définiton de la periode d'échantillonage
    Te = 0.001
    
    # vecteurs et matrices de sauvegarde 
    t = np.zeros(N)
    yy = np.zeros(N)
    # vecteur d'état estimée à l'instant k = 0
    
    x_hat = np.array([[2], [0], [0]])
    
    # entrée echelon
    u = 1.0
    for i in range(0, N) :
        y = system(Te*i, u)
        yy[i]=y
        t[i]=Te*i

    
    
    xx_hat = np.zeros((3,N))
        
    temp = x_hat
    
    for i in range(0, N) :
        temp = observateur(u, yy[i], temp)
        xx_hat[0][i] = temp[0][0]
        xx_hat[1][i] = temp[0][1]
        xx_hat[2][i] = temp[0][2]
    
    y_estimée = xx_hat[0] + xx_hat[1] + xx_hat[2]
    
    plt.figure()
    plt.plot(t,yy)
    plt.plot(t,y_estimée,'r--')
    plt.legend(['y(t)', 'y(t)_estimé'])
    plt.title('Tracé de y(t) et y(t)_estimé')
    plt.xlabel('temps (s)')
    plt.ylabel('Amplitude')
    plt.grid()
    plt.show()
    """
        on voit que notre systeme observateur est stable et converge vers le système reél ce qui nous montre que notre représentation équivalente du système original
        est juste
    """
    
    plt.figure()
    plt.subplot(3,1,1)
    plt.title("Tracé des élement du vecteur d'état éstimé")
    plt.plot(t,xx_hat[0])
    plt.grid()
    plt.ylabel("X1(t)")
    plt.subplot(3,1,2)
    plt.plot(t,xx_hat[1])
    plt.ylabel("X2(t)")
    plt.grid()
    plt.subplot(3,1,3)
    plt.plot(t,xx_hat[2])
    plt.ylabel("X3(t)")
    plt.xlabel("temps(s)")
    plt.grid()
    plt.tight_layout()
    """
        Les trois sous-figure du vecteur d'état sont stable et converge pour nous donnée un résultat final égale au système réel
    """
