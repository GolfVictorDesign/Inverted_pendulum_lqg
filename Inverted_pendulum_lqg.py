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
    
    It has 2 brushed DC motors with an IMU on top to measure theta and encoders on the motors
    the DC motors are controlled by a PWM controller.        

    The states representing the system:
     * x                => Position of the motor-wheels subsystem 
     * x_dot            => Linear speed of the motor-wheels subsystem 
     * theta            => Angle of the pendulum
     * theta_dot        => Angular velocity of the pendulum 
     
     States fixed points :
     * X1 = x           =>  arbitrary fixed point
     * X2 = x_dot       =>  0 
     * X3 = theta       =>  0 for pendulum up, PI for pendulum down
     * X4 = theta_dot   =>  0
"""
# states initial conditions
x_init = 0.0            # initial position of the cart
x_dot_init = 0.0        # initial velocity of the cart
theta_init = 0.0        # initial angle of the pendulum
theta_dot_init = 0.0   # initial angular velocity of the pendulum
states_init = np.array([[x_init], [x_dot_init], [theta_init], [theta_dot_init]])

# physical parameters
m_arm = 0.20        # mass of the pendulum
M_cart = 0.30       # mass of the cart
arm_length = 0.15   # arm length from the cart to the center of mass of the pendulum
g = 9.81            # gravity constant
arm_inertia = (1/3) * m_arm * (arm_length ** 2)     # inertia of an arm around it extremity

Kp = 2.0
Kr = 1.0
beta = -0.5
gamma = 0.01

# matrix values
A43 = -(g/arm_length)*np.cos(theta_init)

dt_system = 0.01

# system matrices
A_matrix = np.array([[0., 1., 0., 0.],
                     [0., beta, 0., 0],
                     [0., 0., 0., 1.],
                     [0., 0., A43, -gamma]])
B_matrix = np.array([[0.], [Kp], [0.], [Kr]])
C_matrix = np.array([[1., 1., 1., 1.]])
D_matrix = np.array([[0.]])


"""
System checks
    Controllability =>  The system is controllable if the rank of its Controllability matrix is equal to the nb o states
    Observability   =>  The system is observable if the rank of its observability matrix is equal to the nb o states
"""
# check controllability
inverted_pendulum_controllable = np.linalg.matrix_rank(control.ctrb(A_matrix, B_matrix))
if A_matrix.shape[1] == inverted_pendulum_controllable:
    message = "The system is controllable, its controllability matrix is full rank " \
              "(rank = {rank:d}, Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))
else:
    message = "The system is NOT controllable, its controllability matrix has rank " \
              "{rank:d} but numbers of states are {states:d})"
    print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))

# check observability
inverted_pendulum_observable = np.linalg.matrix_rank(control.obsv(A_matrix, C_matrix))
if A_matrix.shape[1] == inverted_pendulum_observable:
    message = "The system is observable, its observability matrix is full rank " \
              "(rank = {rank:d}, Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))
else:
    message = "The system is NOT observable, its observability matrix has rank " \
              "{rank:d} but numbers of states are {states:d})"
    print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))


"""
Linear quadratic regulator
"""
# compute the linear quadratic regulator
Q_lqr = np.diagflat([1., 1., 1., 1.])
R_lqr = np.array([1.])
K_lqr, S_lqr, E_lqr = control.lqr(A_matrix, B_matrix, Q_lqr, R_lqr)
