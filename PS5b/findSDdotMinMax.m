function [sDdotMin,sDdotMax,valid] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot)
%Given acceleration constraints, qDdotMax, Bvec, Mvec and sdot, this
% function computes the maximum and minimum viable sddot (d^2 s/dt^2).
% The returned value “valid” should be 1 if sDdotMin<sDdotMax, and 0
% otherwise (i.e. if there is no value of sddot within the constraints
% that will keep the hand on the desired path at the specified B(s),
% M(s) and speed)

%Create two sets of constraints (6 constraints total)
sDdot_constr = [(-qDdotMax - Bvec * sdot^2) ./ Mvec;
    (qDdotMax - Bvec * sdot^2) ./ Mvec];

%Find the max and min
sDdotMin = -Inf;
sDdotMax = Inf;
for i = 1:6
    if sDdot_constr(i) <= 0 && sDdot_constr(i) > sDdotMin
        sDdotMin = sDdot_constr(i);
    else if sDdot_constr(i) >= 0 && sDdot_constr(i) < sDdotMax
            sDdotMax = sDdot_constr(i);
        end
    end
end

%Determine if the solution is valid
valid = sDdotMin < sDdotMax;
