%Edward Venator
%EECS 489, Spring 2012
%PS5B

clear all;
clc;

%define the robot:
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

%assemble the above into a DH matrix
DH = [L1, d1, alpha1, theta1;
    L2, d2, alpha2, theta2;
    L3, d3, alpha3, theta3]; %replace 4th column for pose qvec

qdotMax=[2.0;2.0;0.5]; %dynamic constraints on joint velocities
qDdotMax=[10.0;10.0;5]; %dynamic constraints on joint accelerations

%define the hand path:
R = 3.0; %3.0; %also consider R=0.1m

%Pre-allocate vectors
s_vec = 0:.01:6.28;
s_dot = zeros(1,length(s_vec));
M_vec = zeros(3,length(s_vec));
B_vec = zeros(3,length(s_vec));
vel_constr = zeros(1,length(s_vec));
acc_constr = zeros(1,length(s_vec));
sDdotMin = zeros(1,length(s_vec));
sDdotMax = zeros(1,length(s_vec));

%step through s, creating M, B, and SdotMax
for i = 1:length(s_vec)
    [M, B] = computeMvecBvec(DH, R, s_vec(i));
    M_vec(:,i) = M;
    B_vec(:,i) = B;
    [s_dot(i), vel_constr(i), acc_constr(i)] = findSdotMax(qDdotMax, qdotMax, B, M, 10);
    [sDdotMin(i),sDdotMax(i),valid] = findSDdotMinMax(qDdotMax,B,M,2);
end

%Plot Results
figure(1)
plot(s_vec,M_vec(1,:),'b',s_vec,M_vec(2,:),'r',s_vec,M_vec(3,:),'g');
title('Mvec for joints 1 (b), 2 (r), and 3 (g)');

figure(2)
plot(s_vec,B_vec(1,:),'b',s_vec,B_vec(2,:),'r',s_vec,B_vec(3,:),'g');
title('Bvec for joints 1 (b), 2 (r), and 3 (g)');

figure(3)
plot(s_vec,s_dot);
title('Maximum sDot');

figure(4)
plot(s_vec,vel_constr);
title('Maximum sDot (velocity constraint only)');

figure(5)
plot(s_vec,acc_constr);
title('Maximum sDot (acceleration constraint only)');

figure(6);
plot(s_vec, sDdotMax, s_vec, sDdotMin);
title('Maximum (b) and Minimum(g) sDdot for sDot=2');