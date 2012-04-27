%main program for PS7 force-control simulation
clear;
%make these global, so don't have to pass them to functions...
global Kve
global Bl
global Bm
global Kvt
global Ke
global Kpt
global DH

%%%%%%%%%%%%%%% THESE ARE THE ONLY LINES YOU SHOULD EDIT IN THIS PROGRAM

Ke=10;  %"mattress spring" environment stiffness
Kpt=[4000 0; 0 4000]; %transmission springs;

%%%%%%%%%% END OF LINES TO EDIT IN THIS PROGRAM 

Fdes=[1.0;0.0]; %desired contact force of 1N horizontal, 0 N vertical

Bm=[10.1 0
    0 5.1]; %inertias of motors

Bl=[10 0
    0 5]; %inertias of links...very simplified

%environment damping
Kve=1;

%transmission damping
Kvt=[10 0; 0 10];

%set up DH matrix as rows a, d, alpha, theta
L1 = 1;
L2 = 1; 
d1=0.0;
d2=0.0;
alpha1=0;
alpha2=0;
theta1=0; %these values will change
theta2=0;

DH = [L1, d1, alpha1, theta1;
      L2, d2, alpha2, theta2];


%reset the simulator
Simulate_1ms([0;0],true);
%run the simulation for this many time steps (this many milliseconds)
nsteps=10000;

%define some storage locations to hold results
qm_hist=zeros(2,nsteps);
qmdot_hist=zeros(2,nsteps);
ql_hist=zeros(2,nsteps);
qldot_hist=zeros(2,nsteps);
F_hist=zeros(2,nsteps);
p_hist=zeros(2,nsteps);
tau_hist=zeros(2,nsteps);
time_hist=1:nsteps;

%initial values before starting simulation...
Fe=[0;0];  %force on environment
[qm,qmdot,F,ql, qldot]=Simulate_1ms([0;0]); %get initial conditions
DH(:,4)=qm; %initialize the joint angles in DH matrix

%integrate for nsteps time steps...
for i=1:nsteps
    [ Aout, Amats ] = fwd_kin( DH ); %forward kinematics to get hand position
    p_hist(:,i)=Aout(1:2,4);    %store hand positions
    [ J ] = Jacobian( Amats );%compute the Jacobian
    Jp = J(1:2,1:2); %pull out the positional Jacobian, 2x2
    tau=force_controller(Fdes,Fe,Jp,qmdot);  %joint torques are computed by your force-control algorithm
    tau_hist(:,i)=tau; %save these for plotting
    %use the provided simulator to simulate robot dynamices with
    %intermittent contact; save the results in storage arrays
    [ qm_hist(:,i), qmdot_hist(:,i), F_hist(:,i), ql_hist(:,i), qldot_hist(:,i)] = Simulate_1ms( tau);
    Fe = -F_hist(:,i); %force on the environment is opposite of force on the hand
    qmdot = qmdot_hist(:,i);
    DH(:,4)=qm_hist(:,i); %update the joint angles in DH matrix
    envHorizForce=Fe(1);  %provide some output to show progress--display the horizontal force on the environment
end

figure(1);
plot (p_hist(1,:),p_hist(2,:));
xlabel('hand x position')
ylabel('hand y position')
title('hand motion')

figure(2);
%force on the environment is negative of force on the hand
plot(time_hist,-F_hist(1,:),'b',time_hist,-F_hist(2,:),'r');
xlabel('time (ms)')
ylabel('force (N)')
title('force history: horizontal (b) and vertical (r)')

figure(3)
plot(time_hist,ql_hist(1,:),'b',time_hist,ql_hist(2,:),'r')
title('q1 (b) and q2 (r) vs time')
xlabel('time (msec)')
ylabel('angle (rad)')

figure(4)
plot(time_hist,tau_hist(1,:),'b',time_hist,tau_hist(2,:),'r')
xlabel('time (ms)')
ylabel('torque')
title('joint torques vs time')
