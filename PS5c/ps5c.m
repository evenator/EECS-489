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
    s=sVec(i)
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
title('Max sdot (b) and Max sdot for Safe Braking (r) vs s');