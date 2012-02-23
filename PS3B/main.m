%Edward Venator
%EECS 489 Spring 2012
%Assignment 3b

%Given Denavit Hartenburg Parameters and a desired position, find joint
%   angles to achieve that position by using the Jacobian.
% Loads DH parameters into DH
% Loads desired position/orientation transform into T_des

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

%Desired position
T_des = [
    -0.8415 -0.5403 0       3.0714
    0.2919  -0.4546 0.8415  -56.074
    -0.4546 0.7081  0.5403  -45.0033
    0       0       0       1
    ];

%Initialize theta to all zeros
theta = zeros(7,1);

%Calculate position of tool
T = eye(4);
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
%Extract Position and 
p = T(1:3,4);
ea = 
while norm(T_des - T) > thresh
    
end

%Initialize tranformation matrix to identity
T = eye(4);

%Initialize storage matrices for p and z to calculate Jacobian later
p = zeros(3,length(theta));
z = zeros(3,length(theta));


%Calculate Jacobian
J = zeros(6, length(theta));
p_n = T(1:3,4);
for i=1:length(theta)
    %For a revolute joint
    J_pi = cross(z(:,i), p_n-p(:,i));
    J(:,i) = [J_pi;z(:,i)];
end