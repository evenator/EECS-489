function [sdotMax, vel_constr, acc_constr] = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat)
%Finds the maximum viable value of sdot (between 0 and sdotSat, 
% with sdotSat=10) given Bvec, Mvec, and the joint acceleration 
% constraints and joint velocity constraints.
%Modified to also return velocity and acceleration constraints for
%illustrative purposes

%Velocity is constrained by qdotMax / q_s
vel_constr = min(abs(qdotMax ./ Mvec));

%Acceleration is constrained (assuming s_ddot=0) by qDdotMax / B
acc_constr = sqrt( min(abs(qDdotMax ./ Bvec)))

%Minimum constraint dominates
sdotMax = min([ vel_constr, acc_constr, sdotSat]);
end