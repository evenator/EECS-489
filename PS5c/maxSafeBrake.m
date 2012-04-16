function [sdotSafeBrake]=maxSafeBrake(qDdotMax,qdotMax,DH,R,s,sdotMax)
delta = .01;
sdotSafeBrake = sdotMax;
this_delta = sdotSafeBrake;
%Short circuit if we're already safe
if brakeToRest(qDdotMax,qdotMax,DH,R,s,sdotSafeBrake) == 1
    return
end
%Binary search
while this_delta > delta
    if brakeToRest(qDdotMax,qdotMax,DH,R,s,sdotSafeBrake) < 1
        %Cannot brake, so lower sdotSafeBrake
        this_delta = this_delta / 2;
        sdotSafeBrake = sdotSafeBrake - this_delta
    else
        %Can brake, so raise sdotSafeBrake
        this_delta = this_delta / 2;
        sdotSafeBrake = sdotSafeBrake + this_delta
    end
end