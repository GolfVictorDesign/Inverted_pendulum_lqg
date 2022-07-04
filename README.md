# Inverted_pendulum_lqg

Build with the help of Steve Brunton's Control Bootcamp :
https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m
and David Deley's inverted pendulum studies :
https://daviddeley.com/pendulum/pendulum.htm
and https://www3.diism.unisi.it/~control/ctm/examples/pend/invpen.html

## State-space model:

The system is a self-balancing robot as we can see many on the internet.
    
It has 2 brushed DC motors with an IMU on top to measure theta and encoders on the motors
the DC motors are controlled by a PWM controller.

The equations of motion from the lagrangian equation yields:
```
(M + m).x_dotdot - m.l.theta_dot.cos(theta) + m.l.theta_dot².sin(theta) = F
l.theta_dot - g.sin(theta) = x_dotdot.cos(theta)
```

The pendulum 


The states representing the system:
 * x                => Position of the motor-wheels subsystem 
 * x_dot            => Linear speed of the motor-wheels subsystem 
 * theta            => Angle of the pendulum
 * theta_dot        => Angular velocity of the pendulum 
 
States fixed points
 * X1 = x           =>  arbitrary fixed point
 * X2 = x_dot       =>  0 
 * X3 = theta       =>  0 for pendulum up
 * X4 = theta_dot   =>  0
 
```
        [                   X2                   ]   [    x_dot   ]
X_dot = [             -beta.X2 + Kp.u            ] = [  x_dotdot  ]
        [                   X4                   ]   [  theta_dot ]
        [-(g/l).sin(X3) - gamma.X4 + Kq.X2 - Kr.u]   [theta_dotdot]
```

beta  => mechanical loss and damping of the motor-wheels subsystem 
Kp    => 'gain' of input (i.e voltage) to acceleration of the motor-wheels subsystem
Kq    =>  
Kr    => 
gamma => Damping of the pendulum
         
A is the Jacobian of X :
```
A11 = dX1/dX1 = 0   A12 = dX1/dX2 = 1    A13 = dX1/dX3 = 0   A14 = dX1/dX4 = 0
A21 = dX2/dX1 = 0   A22 = dX2/dX2 = beta A23 = dX2/dX3 = 0   A24 = dX1/dX4 = 0
A31 = dX3/dX1 = 0   A32 = dX3/dX2 = 0    A33 = dX3/dX3 = 0   A34 = dX1/dX4 = 1
A41 = dX4/dX1 = 0   A42 = dX4/dX2 = Kq   A43 = dX4/dX3 = A43 A44 = dX4/dX4 = -gamma
```
A43 = -(g/l).cos(X3)

```
    | 0     1      0      0  |
A = | 0   -beta    0      0  |
    | 0     0      0      1  |
    | 0     Kq    A43  -gamma|
```   
B is the control matrix :

u => PWM voltage controller of the motor 
let assume when PWM=100% u = 10V

```
    [ 0  ]
B = [ Kp ]
    [ 0  ]
    [ Kr ]
```