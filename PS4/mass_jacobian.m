function [ J ] = mass_jacobian( p, z,  n)
%mass_jacobian( pos_vec, joint_number)
% Calculates the Jacobian for the mass at a given joint number
% p is the list of all joint positions in the base/world coordinate
% system
% z is a list of all joint z-axis unit vectors in the base/world coordinate
% system
% n is the number of the joint to which the mass is fixed

%Calculate Jacobian
J = zeros(3, size(p,2));
p_n = p(:,n+1);
for i=1:n
    %For a revolute joint
    J(:,i) = cross(z(:,i), p_n-p(:,i));
end

end