%Edward Venator
%EECS 489 Spring 2012
%Assignment PS5C

clear all
clc
%set up DH matrix as rows a, d, alpha, theta
L1 = 0;
L2 = 5;
L3 = 4;
d1=0.5;
d2=0;
d3=0;
alpha1=pi/2;
alpha2=0;
alpha3=0;
theta1=0; %these values will change
theta2=0;
theta3=0;

qdotMax=[2.0;2.0;0.5];
qDdotMax=[10.0;10.0;5];

DH = [L1, d1, alpha1, theta1;
    L2, d2, alpha2, theta2;
    L3, d3, alpha3, theta3];
R = 3.0;

sVec = 0:0.01:2*pi; %sample values of s from 0 to 2pi in steps of 0.01
npts=length(sVec);

Mvecs = zeros(3,npts);
Bvecs = zeros(3,npts);

sdotMaxes=zeros(1,npts);
sdotBrakeMaxes=zeros(1,npts);
sdotSat=10; %only bother searching accel constraint up to this value of sdot

for i=1:npts
    s=sVec(i);
    [Mvec,Bvec] = computeMvecBvec(DH,R,s);
    Mvecs(:,i)=Mvec;
    Bvecs(:,i)=Bvec;
    %compute the velocity constraint for each sample of s
    sdotMax = findSdotMax(qDdotMax,qdotMax,Bvec,Mvec,sdotSat);
    sdotBrakeMax = maxSafeBrake(qDdotMax,qdotMax,DH,R,s,sdotMax);
    sdotMaxes(i)=sdotMax;
    sdotBrakeMaxes(i) = sdotBrakeMax;
end

%Plot Results
figure(1)
plot(sVec,sdotMaxes,'b',sVec,sdotBrakeMaxes,'r');
title('Max sdot (b),Max sdot for Safe Braking (r), and Optimal sdot (g) vs s');

%Simulate Braking
sdot = 0; %At the end of braking, sdot must = 0
dt = .01; %simulation step size
for i=npts:1 %step through backwards
    [sDdotMin, ~, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot); %Find the max braking speed
    s=s-sdot*dt; %integrate bkwds in time
    sdot=sdot-sddot*dt; %integrate bkwds in time
    if sdot >= sdotBrakeMaxes(i)
        s_brake = s;
        %If the braking curve intersects the optimal velocity, this is the
        %brake point and stop simulating.
        break;
    end
end

%Simulate Braking
s = 6.28; %Braking ends at 2pi
sdot = 0; %At the end of braking, sdot must = 0
dt = .01; %simulation step size
while s>0 %step through backwards
    %Find the max braking speed
    [Mvec,Bvec] = computeMvecBvec(DH,R,s);
    [sDdotMin, ~, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,sdot);
    s=s-sdot*dt; %integrate bkwds in time
    sdot=sdot-sDdotMin*dt; %integrate bkwds in time
    i = round(s / 6.28 * npts); %Find closest index
    if sdot >= sdotBrakeMaxes(i)
        s_brake = s;
        %If the braking curve intersects the optimal velocity, this is the
        %brake point and stop simulating.
        break;
    end
end

%Optimal Control
%Initialize s, s_dot, s_2dot to 0
s = 0;
s_dot = 0;
s_2dot = 0;

dt = .001; %Simulation time step
i = 0; %Counter
thresh = .01; %Threshold for sliding mode controller
while s<6.28
    i = i + 1; %increment counter
    %Find the max accelerations
    [Mvec,Bvec] = computeMvecBvec(DH,R,s);
    [s_2dot_min, s_2dot_max, ~] = findSDdotMinMax(qDdotMax,Bvec,Mvec,s_dot);
    if s < s_brake
        %Not braking yet
        index = round(s / 6.28 * npts); %Find closest index for comparison against sBrakeMaxes
        if(index == 0)
            index = 1;
        end
        %Sliding mode control
        if s_dot < (sdotBrakeMaxes(index) - thresh)
            s_2dot = s_2dot_max;
        elseif s_dot > (sdotBrakeMaxes(index) + thresh)
            s_2dot = s_2dot_min;
        else
            s_2dot = (s_2dot_min + s_2dot_max)/2 + (s_2dot_max - s_2dot_min) * (s_dot - sdotBrakeMaxes(index)); %linear interpolation
        end
    else
        %Braking
        if(s_dot>0)
            s_2dot = s_2dot_min;
        else
            break; %Keeps from going backwards in s
        end
    end
    %Simulate s_dot and s
    s_dot = s_dot + dt * s_2dot;
    s = s + dt * s_dot;
    
    %Store things for plots. This is inefficient, but hey, whatever
    s_optimal(i) = s;
    s_dot_optimal(i) = s_dot;
    
    %Calculate and store q_dot and q_2dot
    [p, p_s, p_ss] = nomHand(s,R);
    p_dot = p_s * s_dot;
    p_2dot = p_ss * s_2dot;
    %Calculate Jacobian for inverse kinematics
    DH(:,4) = fncInvKinPS5(p, DH);
    [~, A_mats] = fwd_kin( DH );
    J = Jacobian( A_mats);
    Jp = J(1:3, :);
    q_dot = (p_dot' / Jp')';
    q_2dot = (p_2dot' / Jp')';
    q_dot_optimal(:,i) = q_dot;
    q_2dot_optimal(:,i) = q_2dot;    
end

%Plot s_optimal vs s
hold on;
plot(s_optimal, s_dot_optimal, 'g');
hold off;

%Plot Joint velocities
figure(2);
plot(s_optimal, q_dot_optimal(1,:), 'r', s_optimal, q_dot_optimal(2,:), 'g', s_optimal, q_dot_optimal(3,:), 'b');
title('Joint 1 (r), 2(g), and 3(b) velocities vs s');

%Plot Joint accelerations
figure(3);
plot(s_optimal, q_2dot_optimal(1,:), 'r', s_optimal, q_2dot_optimal(2,:), 'g', s_optimal, q_2dot_optimal(3,:), 'b');
title('Joint 1 (r), 2(g), and 3(b) accelerations vs s');