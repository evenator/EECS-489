function sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat)
% must have -qDdotMax(i) <= Mvec(i)sddot + Bvec(i)sdot^2 <= qDdotMax(i)
% must apply to all 3 jnts
% find max sdot for which this is possible--i.e., just as lower bnd touches
% upper bnd
% search only positive values of sdot up to sdotSat

%compute the sdot constraints based on joint velocities alone:
    %joint-velocity constraints
    q_s = Mvec; %synonym for q_s
    aqdotsMax = abs(qdotMax./q_s);
    sdotMaxVel = min(aqdotsMax); %binding speed constraint limits max sdot(s)

%now, compute sdot limits due to sddot and qDdot constraints
%brute force...try increasing values of sdot and save the
%largest value that satisfies the constraints
nsamps=1000;
sdot=0.0;
valid=1;
dsdot=sdotSat/nsamps;
while (sdot<sdotSat)&&(valid>0.5)
    sdot=sdot+dsdot;
    [sDdotMin,sDdotMax,valid] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot);
end
sdotMaxAcc=sdot;  %this was the first value of sdot that violated qddot constraints
%check whether velocity constraint or accel constraint is more restrictive:
sdotMax = sdotMaxVel;
if sdotMaxAcc < sdotMaxVel
    s_velLimAccVel=[sdotMaxAcc,sdotMaxVel] %debug output...found accel is limiting, not vel constraint
    sdotMax = sdotMaxAcc;
end
    
 
end

