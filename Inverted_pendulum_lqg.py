"""
Created on 2022-06-18
@author: GolfVictorDesign

Simulation of a LQG control for inverted pendulum on wheels
"""

import numpy as np
import control
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import random
from collections import deque

""" 
State-Space model :

    The system is a self balancing robot as we can see many on the internet.
    
    It has 2 brushed DC reducted motors with an IMU on top to measure theta and encoders on the motors
    the DC motors are controlled by a PWM controller.

    As a simplified system, let say we have 4 states 
     * x            => Position of the motor-wheels subsystem 
     * x_dot        => Linear speed of the motor-wheels subsystem 
     * theta        => Angle of the pendulum
     * theta_dot    => Angular velocity of the pendulum 
     
    States                 Fixed points
    X1 = x             =>  arbitrary fixed point
    X2 = x_dot         =>  0 
    X3 = theta         =>  PI
    X4 = theta_dot     =>  0
     
            [                   X2                   ]   [    x_dot   ]
    X_dot = [             -beta.X2 + Kp.u            ] = [  x_dotdot  ]
            [                   X4                   ]   [  theta_dot ]
            [-(g/l).sin(X3) - gamma.X4 + Kq.X2 - Kr.u]   [theta_dotdot]
            
            beta  => mechanical loss and damping of the motor-wheels subsystem 
            Kp    => 'gain' of input (i.e voltage) to acceleration of the motor-wheels subsystem
            Kq    =>  
            Kr    => 
            gamma => Damping of the pendulum
             
    A is the Jacobian of X :
    
    A11 = dX1/dX1 = 0   A12 = dX1/dX2 = 1    A13 = dX1/dX3 = 0   A14 = dX1/dX4 = 0
    A21 = dX2/dX1 = 0   A22 = dX2/dX2 = beta A23 = dX2/dX3 = 0   A24 = dX1/dX4 = 0
    A31 = dX3/dX1 = 0   A32 = dX3/dX2 = 0    A33 = dX3/dX3 = 0   A34 = dX1/dX4 = 1
    A41 = dX4/dX1 = 0   A42 = dX4/dX2 = Kq   A43 = dX4/dX3 = A43 A44 = dX4/dX4 = -gamma
    
    A43 = -(g/l).cos(X3)
    
        [ 0     1     0      0  ]
    A = [ 0   -beta    0      0  ]
        [ 0     0     0      1  ]
        [ 0     Kq    A43  -gamma]
        
    B is the control matrix :
    
    u => PWM voltage controller of the motor 
    let assume when PWM=100% u = 10V
    
        [ 0  ]
    B = [ Kp ]
        [ 0  ]
        [ Kr ]
"""
theta0 = np.pi
m = 0.1
l = 0.15
g = m * 9.81
Kp = 2.0
Kr = 1.0
beta = -0.5
gamma = 0.01
A43 = -(g/l)*np.cos(theta0)

dt_system = 0.01

A_matrix = np.array([[0., 1., 0., 0.],
                     [0., beta, 0., 0],
                     [0., 0., 0., 1.],
                     [0., 0., A43, -gamma]])

B_matrix = np.array([[0.], [Kp], [0.], [Kr]])

C_matrix = np.array([[0., 1., 0., 1.]])

D_matrix = np.array([[0.]])

system_ss = control.ss(A_matrix, B_matrix, C_matrix, D_matrix)
print(system_ss)

discrete_system_ss = control.c2d(system_ss, dt_system)
print(discrete_system_ss)

inverted_pendulum_controllable = np.linalg.matrix_rank(control.ctrb(A_matrix, B_matrix))

if A_matrix.shape[1] == inverted_pendulum_controllable:
    message = "The system is controllable, its controllability matrix is full rank " \
              "(rank = {rank:d} Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))

inverted_pendulum_observable = np.linalg.matrix_rank(control.obsv(A_matrix, C_matrix))

if A_matrix.shape[1] == inverted_pendulum_observable:
    message = "The system is observable, its observability matrix is full rank " \
              "(rank = {rank:d} Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))