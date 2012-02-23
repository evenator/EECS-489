%Edward Venator
%EECS 489 Spring 2012
%Assignment 3a, Problem 2

%Given Denavit Hartenburg Parameters, joint angles, and perturbed joint
%angles, calculated the change in position for the perturbation to compare
%with the results of the Jacobian calculated in problem1.

%Run problem1 script to calculate Jacobian.
% Loads DH parameters into DH
% Loads Jacobian into J
% Loads angles into theta
% Loads transformation matrix into T
problem1;

%Perturbed theta list
theta2 = [1.001 2.001 1.5001 2.001 -1.001 2.001 3.001];

%Initialize tranformation matrix to identity
T2 = eye(4);

%Create transformation matrix
for i=1:length(theta)
    %Pull alpha, theta, a, and d from DH table
    alpha_i = DH(i,1);
    theta_i = theta2(i);
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
    T2 = T2 * transform;
end

%Extract Positions
P1 = T(1:3,4);
P2 = T2(1:3,4);

%Calculate difference in position
deltaP = P2 - P1

%Calculate dP with the Jacobian
dTheta = theta2 - theta;
dP = J * dTheta'

%dP(1:3) should ~= deltaP