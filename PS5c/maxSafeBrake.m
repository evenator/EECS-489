function [sdotSafeBrake]=maxSafeBrake(qDdotMax,qdotMax,DH,R,s,sdotMax)
%maxSafeBrake returns the maximum sdot for which the robot can safely
%brake to rest, given s

delta = .01; %Specifies the precision of the result.
sdotSafeBrake = sdotMax; %Initialize sdot to max value
this_delta = sdotSafeBrake; %This_delta is the amount to increment or decrement sdotSafeBrake by during binary search. It is halved each iteration.

%Short circuit if we're already safe. This saves a lot of running time
if brakeToRest(qDdotMax,qdotMax,DH,R,s,sdotSafeBrake) == 1
    return
end

%Binary search
while this_delta > delta %End if the delta is less than the specified precision
    this_delta = this_delta / 2; %Binary search, so divide the increment decrement quantity by two each time
    if brakeToRest(qDdotMax,qdotMax,DH,R,s,sdotSafeBrake) < 1
        %Cannot brake, so lower sdotSafeBrake
        sdotSafeBrake = sdotSafeBrake - this_delta;
    else
        %Can brake, so raise sdotSafeBrake
        sdotSafeBrake = sdotSafeBrake + this_delta;
    end
end