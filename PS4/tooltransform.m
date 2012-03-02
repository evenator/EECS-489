function [ T, p, z ] = tooltransform( theta, DH )
%Calculates the position and orientation of the tool and returns it as a
%homogeneous tranform matrix
%Also returns the list of position and z-axis vectors for each joint

%Calculate position of tool
T = eye(4);
%Initialize storage matrices for p and z to calculate Jacobian later
p = zeros(3,length(theta)+1);
z = zeros(3,length(theta));
for i=1:length(theta)
    %Store position and z vector for Jacobian
    p(:,i) = T(1:3,4);
    z(:,i) = T(1:3,3);
    %Pull alpha, theta, a, and d from DH table
    a_i = DH(i,1);
    theta_i = theta(i);
    alpha_i = DH(i,2);
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
p(:,i+1) = T(1:3,4);
end

