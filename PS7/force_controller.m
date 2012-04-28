function [tau, F_u, F_err, F_des]=force_controller(F_des,F_env,J_p,q_m_dot)
%EDIT THIS FUNCTION TO COMPUTE AN INTELLIGENT tau
%force_controller: given desired force on environment,
%  given measured force on environment,
%  given positional Jacobian
%  given motor angular velocities
%  compute a pair of joint torques, tau, to exert force control to
%  achieve the desired environment force

%Force Error Feedback
k_pf = [200 0;
    0 1.5];
%Velocity Damping
k_vf = [1000 0;
    0 15];
F_err = F_des - F_env;
F_u = F_des + k_pf * F_err;
tau = J_p' * F_u - k_vf * q_m_dot;
end

