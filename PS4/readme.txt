Edward Venator
EECS 489 Spring 2012
Problem Set 4

Algorithms are documented within the attached matlab code.
problem1.m is the main file for problem 1, parts A and B.
problem2.m is the main file for problem 2.
tooltransform.m is used in problems 1 and 2 to calculate the tool position
	and the position and z-axis unit vector of each joint.
mass_jacobian.m is used in problems 1 and 2 to calculate the Jacobians of a 
	mass a specified joint with respect to all joints between itself and the 
	base.
joint_torques.m is used in problem 2 to calculate the net torque on a joint due
	to gravity and the springs attached to the joints.


Solutions:
1a (N-m)
    0.3920
  985.8800
    0.3920
  248.9200
    0.3920
    1.9600
    0.3920
    
1b (N-m)
 -762.2332
  203.3078
 -145.3988
 -168.2441
    0.5129
   -0.3256
    0.1235

2 (radians)
    0.0000
    0.0986
    0.0000
    0.0249
    0.0000
    0.0002
    0.0000