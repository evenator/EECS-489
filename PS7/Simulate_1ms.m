function [ qm qmdot F ql qldot] = Simulate_1ms( tau,reset )
%SIMULATE this function integrates the motor and link dynamics over a 1ms
%timestep for a given set of motor torques
%uses a 100usec time  step for Euler integration

global Bl %matrix inertia tensor links
global Bm %matrix inertia tensor of motors
global Kpt %transmission stiffness
global Kvt %transmission damping
global Ke %environment stiffness
global Kve %environment damping
global DH %DH parameters

%persistent variables retain their values between function calls
%the "i" suffix means these are internal
%when done w/ simu, copy these to ql, qldot, etc for return values
persistent qli %link angles
persistent qldoti %link angular velocities
%motor angles and angular velocities:
persistent qmi %i indicates internal and allows use of qm outside this fuction
persistent qmdoti
dt=.0001;

if (nargin<2)
    reset=false;
end
if (isempty(qli)) 
    reset=true;
end
%trick to reset persistent variables to initial conditions
if (reset) %assign initial conditions
    pvec=[1.199;1.2]; %initialize simulation from this hand pose
    qli = fncInvKinPS7(pvec,DH); %find corresponding link angles
    qmi = qli; %initialize motor angles to be same as joint angles
    qmdoti=[0;0]; %initialize links and motors at rest
    qldoti=qmdoti;
end
for i=1:10 %break up 1ms simu into ten smaller time steps

    %Find the pose and velocity of the robot using ql 
    DH(:,4)=qli; %update the joint angles in DH matrix
    [ Aout, Amats ] = fwd_kin( DH ); %forward kinematics
    pvec=Aout(1:2,4);
    
    [ J ] = Jacobian( Amats );%compute the Jacobian
    Jp = J(1:2,1:2); %pull out the positional Jacobian, 2x2
    
    Pdot=Jp*qldoti; %hand velocity
    
    %Find the force of the environment on the hand based on the hand position and
    %velocity
    if (pvec(1)>1.2) 
        F(1,:)=-Ke*(pvec(1)-1.2) - Kve*Pdot(1);
        F(2,:)=-Kve*Pdot(2);
    else
        F=[0;0];
    end

  
   %motor dynamics: motor torques and transmission springs/dampers act on
   %motor inertias
  qmdotdot=inv(Bm)*(tau+Kpt*(qli-qmi)+Kvt*(qldoti-qmdoti));
   %link accels, from robot dynamics, ignoring gravity and Coriolis/centrifugal effects
   %efforts on links are due to endpoint forces as well as transmission
   %torques
  qldotdot=inv(Bl)*( Kpt*(qmi-qli)+Kvt*(qmdoti-qldoti) + Jp'*F);
  
  %integrate motor and link states forward in time:
  qmi=qmi+qmdoti*dt; %integrate motor vel's
  qmdoti=qmdoti+qmdotdot*dt; % integrate motor accels

  qli=qli+qldoti*dt;
  qldoti=qldoti+qldotdot*dt; 
end
ql=qli;
qldot=qldoti;
qm=qmi;
qmdot=qmdoti;

