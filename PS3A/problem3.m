%Edward Venator
%EECS 489 Spring 2012
%Assignment 3a, Problem 3

%Given Denavit Hartenburg Parameters and joint angles calculate the
%Jacobian.
% Loads DH parameters into DH
% Loads Jacobian into J
% Loads angles into theta
% Loads transformation matrix into A

DH = [
    -pi/2   0.0     0;
    -pi/2   0.0     0;
    pi/2    4.2     0;
    pi/2    4.2     0;
    pi/2    1.9     38;
    pi/2    1.75    0;
    0.0    0.0      4.5;
    ];

theta = [pi/2 2*pi 1 1 pi pi 2];
A = eye(4);
for i=1:length(theta)
    alpha_i = DH(i,1);
    theta_i = theta(i);
    a_i = DH(i,2);
    d_i = DH(i,3);
    transform = [
        cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
        sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
        0, sin(alpha_i), cos(alpha_i), d_i;
        0, 0, 0, 1;
        ];
    A = A * transform;
end

%Calculate Jacobian