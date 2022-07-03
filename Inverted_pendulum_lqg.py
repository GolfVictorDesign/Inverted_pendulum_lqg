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
        
    The equations of motion from the lagrangian equation yields:
        (M + m).x_dotdot - m.l.theta_dot.cos(theta) + m.l.theta_dot².sin(theta) = F
        l.theta_dot - g.sin(theta) = x_dotdot.cos(theta)

    The states representing the system:
     * x                => Position of the motor-wheels subsystem 
     * x_dot            => Linear speed of the motor-wheels subsystem 
     * theta            => Angle of the pendulum
     * theta_dot        => Angular velocity of the pendulum 
     
    States                  Fixed points
     * X1 = x           =>  arbitrary fixed point
     * X2 = x_dot       =>  0 
     * X3 = theta       =>  0 for pendulum up
     * X4 = theta_dot   =>  0
     
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

# states initial conditions
x_init = 0.0            # initial position of the cart
x_dot_init = 0.0        # initial velocity of the cart
theta_init = 0.0        # initial angle of the pendulum
theta_dot_init = 0.01   # initial angular velocity of the pendulum
states_init = np.array([[x_init], [x_dot_init], [theta_init], [theta_dot_init]])

# physical parameters
m_arm = 0.20        # mass of the pendulum
M_cart = 0.30       # mass of the cart
arm_length = 0.15   # arm length from the cart to the center of mass of the pendulum
g = 9.81            # gravity constant
arm_inertia = (1/3) * m_arm * (arm_length ** 2)

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

# check observability
inverted_pendulum_controllable = np.linalg.matrix_rank(control.ctrb(A_matrix, B_matrix))
if A_matrix.shape[1] == inverted_pendulum_controllable:
    message = "The system is controllable, its controllability matrix is full rank " \
              "(rank = {rank:d}, Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))
else:
    message = "The system is NOT controllable, its controllability matrix has rank " \
              "{rank:d} but numbers of states are {states:d})"
    print(message.format(rank=inverted_pendulum_controllable, states=A_matrix.shape[1]))

# check controllability
inverted_pendulum_observable = np.linalg.matrix_rank(control.obsv(A_matrix, C_matrix))
if A_matrix.shape[1] == inverted_pendulum_observable:
    message = "The system is observable, its observability matrix is full rank " \
              "(rank = {rank:d}, Nb states = {states:d})"
    print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))
else:
    message = "The system is NOT observable, its observability matrix has rank " \
              "{rank:d} but numbers of states are {states:d})"
    print(message.format(rank=inverted_pendulum_observable, states=A_matrix.shape[1]))

# compute the linear quadratic regulator
Q_lqr = np.diagflat([1., 1., 1., 1.])
R_lqr = np.array([1.])
K_lqr, S_lqr, E_lqr = control.lqr(A_matrix, B_matrix, Q_lqr, R_lqr)
