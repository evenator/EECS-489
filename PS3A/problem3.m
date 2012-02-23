%Edward Venator
%EECS 489 Spring 2012
%Assignment 3a, Problem 3

%Given Denavit Hartenburg Parameters and joint angles calculate the
%Jacobian.
% Loads DH parameters into DH
% Loads Jacobian into J
% Loads angles into theta
% Loads transformation matrix into T

%DH Table
DH = [
    -pi/2   0.0     0;
    -pi/2   0.0     0;
    pi/2    4.2     0;
    pi/2    4.2     0;
    pi/2    1.9     38;
    pi/2    1.75    0;
    0.0    0.0      4.5;
    ];

%Theta List
theta = [pi/2 2*pi 1 1 pi pi 2];

%Initialize tranformation matrix to identity
T = eye(4);

%Initialize storage matrices for p and z to calculate Jacobian later
p = zeros(3,length(theta));
z = zeros(3,length(theta));

%Create transformation matrix
for i=1:length(theta)
    %Store position and z vector for Jacobian
    p(:,i) = T(1:3,4);
    z(:,i) = T(1:3,3);
    %Pull alpha, theta, a, and d from DH table
    alpha_i = DH(i,1);
    theta_i = theta(i);
    a_i = DH(i,2);
    d_i = DH(i,3);
    %Calculate this frame's tranformation matrix
    transform = [
        cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
        sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
        0, sin(alpha_i), cos(alpha_i), d_i;
        0, 0, 0, 1;
        ];
    %Multiply this frame's tranformation into the total transformation
    T = T * transform;
end

%Calculate Jacobian
J = zeros(6, length(theta));
p_n = T(1:3,4);
for i=1:length(theta)
    %For a revolute joint
    J_pi = cross(z(:,i), p_n-p(:,i));
    J(:,i) = [J_pi;z(:,i)];
end

J