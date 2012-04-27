function tau=force_controller(F_des,F_env,J_p,q_m_dot)
%EDIT THIS FUNCTION TO COMPUTE AN INTELLIGENT tau
%force_controller: given desired force on environment,
%  given measured force on environment,
%  given positional Jacobian
%  given motor angular velocities
%  compute a pair of joint torques, tau, to exert force control to
%  achieve the desired environment force

k_pf = [10 0;
    0 10];
%feedback terms--proportional to force error
k_vf = [1 0;
    0 1];%feedback terms--proportional to hand velocity

tau = k_pf * (F_des - F_env)+ k_vf * q_m_dot; %this is stupid--do something more intelligent
end

