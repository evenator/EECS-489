function [canBrake] = brakeToRest(qDdotMax,qdotMax,DH,R,s,sdot)
%brakeToRest returns 1 if the robot can safely brake to rest from the given
% initial conditions or 0 otherwise

dt = .01; %10 ms timestep for integration
sdotSat=10; %Maximum sdot
canBrake = 0; %Default canBrake to 0

%Compute sDdotMin and sdotMax for first step
[Mvec,Bvec] = computeMvecBvec(DH,R,s);
[sDdotMin, ~, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot);
sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat);

while sdot >= 0 && sdot <= sdotMax %terminate if we stop (success) or hit the max (failure)
    %Simulate s and sdot
    s = s + sdot * dt;
    sdot = sdot + sDdotMin  * dt;
    %If we stop, success
    if sdot < 0
        canBrake = 1;
    end
    %Compute sDdotMin and sdotMax for next step
    [Mvec,Bvec] = computeMvecBvec(DH,R,s);
    [sDdotMin, ~, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot);
    sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat);
end