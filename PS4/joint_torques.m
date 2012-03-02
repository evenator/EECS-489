function [ tau ] = joint_torques(theta, p, z , m, g )
%Calculates the net torques of all joints

%Gravity Component
%For each joint j, calculate the sum of torques due to all masses i
%There's a much more efficient way to do this, but I can't be bothered
tau = zeros(7,1);
for j = 1:7
    for i = 1:8
        J = mass_jacobian(p, z, i);
        tau(j) = tau(j) + m(i) * g * J(:,j);
    end
end

%Spring Component
%Remove the pi/2 radian offset from the last link, which was created as a
%convenience
theta(7) = theta(7) - pi/2;
tau = tau - theta(1:7) * 10000;
end

