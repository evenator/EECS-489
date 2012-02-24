function [ T, J ] = tooltransform( theta, DH )
%Calculates the position and orientation of the tool and returns it as a
%homogeneous tranform matrix
%Also returns the Jacobian

%Calculate position of tool
T = eye(4);
%Initialize storage matrices for p and z to calculate Jacobian later
p = zeros(3,length(theta));
z = zeros(3,length(theta));
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

end

