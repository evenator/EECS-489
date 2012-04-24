%Simulate( q,qdot,tau)
clear all
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

H=[10, 0
   0, 5]; %inertias; KEEP THESE VALUES
dt=.010; %keep this time step; simulates controller sampling period
nsamps=2000; %number of time steps to simulation...can leave this alone

omega= 100000; %%% VARY THIS VALUE

%compute initial conditions: leave this alone
t = 0; 
[pvec,pdot,pddot]=desHand(t,omega)
q = fncInvKinPS6(pvec,DH) 
DH(:,4)=q;
[ Aout, Amats ] = fwd_kin( DH );
J=  Jacobian( Amats ) 
Jp = J(1:2,1:2); %2x2 positional Jacobian
qdot = inv(Jp)*pdot(1:2);

%define some vectors to hold results
q_hist=zeros(2,nsamps);
qdes_hist=zeros(2,nsamps);
pvec_hist=zeros(2,nsamps);
tau_hist=zeros(2,nsamps);
time=zeros(1,nsamps);

%simulation loop
for i=1:nsamps
    t = dt*i;
    %for current time t, compute desired hand position, vel, accel
    [pvec_des,pdot_des,pddot_des]=desHand(t,omega);
    %inverse kinematics to find corresponding desired joint angles
    qdes = fncInvKinPS6(pvec_des,DH);
    
    %compute the Jacobian to get the desired joint velocities:
    DH(:,4)=qdes;
    [ Aout, Amats ] = fwd_kin( DH );
    J=  Jacobian( Amats );
    Jp = J(1:2,1:2); %2x2 positional Jacobian
    qdot_des = inv(Jp)*pdot(1:2);   
    
    %compute qddot_des = inv(J)*(pddot - Jdot*qdot)
    [ Jp,Jdot ] = JacobiansPS6(DH,qdot_des );
    %resolved motion equations-> q-double-dot desired
    qddot_des=inv(Jp)*(pddot_des(1:2) - Jdot*qdot_des);   
    
    %controller...compute joint efforts as a function of:
    % H: matrix inertia tensor
    % qdes: desired joint angles
    % qdot_des: desired joint velocities
    % qddot_des: desired joint accelerations
    % q: actual joint angles
    % qdot: actual joint velocities
    %%%%%%%%%%%%%  WRITE THIS FUNCTION %%%%%%%%%%%%%%
    tau= controller(H, DH, qdes, qdot_des, qddot_des, q, qdot);
    
    %provided simulation, using robot dynamic model and
    %controller-recommended joint torques:
    [q,qdot]=Simulate(H,DH,q,qdot,tau);
    
    %save the resulting controller torques, desired and actual joint angles
    %and actual hand positions
    tau_hist(:,i)=tau;
    q_hist(:,i)=q;
    qdes_hist(:,i)=qdes;
    DH(:,4)=q;
    [ Aout, Amats ] = fwd_kin( DH );
    pvec_hist(:,i)=Aout(1:2,4); %save the x,y values of the fwd kin (actual hand position)
    time(i)=t;
end

%plot out some results
figure(1);
plot(time,tau_hist(1,:),'b',time,tau_hist(2,:),'r');
grid;
title ('joint torques: jnt1 (b), jnt2 (r)');
xlabel('time (sec)');
ylabel('trq ');

figure(2)
plot(pvec_hist(1,:),pvec_hist(2,:));
grid;
title ('actual hand motion');
xlabel('x');
ylabel('y');
axis([-0.7,0.7, 0,0.5])

figure(3)
plot(time,qdes_hist(1,:),'b',time,q_hist(1,:),'b-.',time,qdes_hist(2,:),'r',time,q_hist(2,:),'r-.')
xlabel('time (sec)')
ylabel('jnt angle (rad)')
title('joint angles: j1 des(b), j1 act(b-.), j2 des(r),j2 act(r-.)')

