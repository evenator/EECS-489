function [canBrake] = brakeToRest(qDdotMax,qdotMax,DH,R,s,sdot)
dt = .01; %10 ms timestep
sdotSat=10;
%Default canBrake to 0
canBrake = 0;

[Mvec,Bvec] = computeMvecBvec(DH,R,s);
[sDdotMin, ~, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot);
sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat);

while sdot >= 0 && sdot <= sdotMax
    s = s + sdot * dt;
    sdot = sdot + sDdotMin  * dt;
    if sdot < 0
        canBrake = 1;
    end
    [Mvec,Bvec] = computeMvecBvec(DH,R,s);
    sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat);
end