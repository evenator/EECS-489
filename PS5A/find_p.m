function [ theta ] = find_p( DH, p_des, theta_init, tolerance )
%Finds the set of theta angles to move the arm with parameters DH to
%position p_des from initial angles theta_init with tolerance

%Find start position
theta = theta_init;
[J, p] = jacobian(DH, theta);


while(norm(p_des-p)>tolerance)
    p_dot = p_des - p;
    theta_dot = p_dot \ J;
    theta = theta + norm(p_dot)*tolerance/10 * theta_dot;
    [J, p] = jacobian(DH, theta);
end
end