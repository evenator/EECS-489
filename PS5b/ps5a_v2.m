%489 ps5, pt 1

clear all
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

%%%%%%%%%%%%%%  RE-USE THE ABOVE FOR PS5B %%%%%%%%%%%%%

%speed is limited by q1dot for R=3 at omega = 2.7473
%speed is limited by q2ddot for R=0.1 at omega = 20.0942

timeScale =  1.0% change this to speed up/slow down motion
omega_nom = 1*timeScale; %start w/ some nominal freq for the circular path
dt = 2*pi/200;
tvec =0:dt:2*pi;
phivec = omega_nom*tvec;
[dummy,npts] = size(tvec);
%compute the desired hand poses:
%x=4, y=1+Rsin(phi), z=1+Rcos(phi)
xvecs = 4*ones(1,npts);
yvecs = 1+R*sin(phivec);
zvecs = 1+R*cos(phivec);
pvecs = [xvecs;yvecs;zvecs];

%init:
qvecDot=zeros(3,1);
qddot=zeros(3,1);
qvecs=[];
qdotvecs=[];
qddotvecs=[];
for i=1:npts
    s=phivec(i);
    sdot=omega_nom;
    sddot=0.0;  %for constant speed, d^2 s/dt^2 = 0
    [pvec,p_s,p_ss]=nomHand(s,R); %compute hand pose at s, as well as derivatives w/rt s
    qvec = fncInvKinPS5(pvec,DH);  %inverse kinematics: p-> q
    qvecs=[qvecs,qvec];   %save for plotting
    
    DH(:,4)=qvec; %assign these solution values to q-column of DH
    [ Aout, Amats ] = fwd_kin( DH ); %compute all the A matrices
    % test inverse kinematics--display fwd kin to confirm
    %Aout %works...fourth column = pvec; uncomment these lines to confirm
    %pvec    
    
    J = Jacobian( Amats ); %use the A-matrices to compute the Jacobian--geometric technique
    Jp = J(1:3,:); %translational part of Jacobian, 3x3
    
    %compute dq/ds:
    q_s = inv(Jp)*p_s; %follows from pdot = J*qdot, and dp/ds = J*dq/ds
    qvecDotOld = qvecDot; %useful to test dq/dt
    qvecDot = q_s*sdot; %qdot = q_s*sdot; qdot scales with sdot, w/ terms proportinal to dq/ds
    qdotvecs=[qdotvecs,qvecDot];  %save for plotting
    
    pvecDot = p_s*sdot; %dp/dt = dp/ds * ds/dt
    pvecDdot = p_ss*(sdot)^2 + p_s*sddot; %general form for d^2p/dt^2
     %note that sddot=0 in this case, so p_s term is unnecessary
    
    %terms J, Jdot, and Hessian at sdot=1, i.e. omega=1
     %Jp computation is redundant w/ above
     %JdotNom is at nominal speed, sdot=1, i.e. Jdot*qdot at omega=1:
     %don't actually use the Hessian here... but could
     %JacobiansPS5 uses analytic computation, not cross products
    [ JpTest,JdotNom,Hijk ] = JacobiansPS5(DH,q_s );
    


    %test Jdot: should find delta-J = Jdot*dt
    %compare to J(q+qdot*dt)-J(q)
    %WORKS
%     dt = 0.001;
%     dJ = Jdot*dt
%     qTest = qvec+qvecDot*dt;
%     DHtest = DH;
%     DHtest(:,4)=qTest;
%     [Jp2,Jdot2]=JacobiansPS5(DHtest,qvecDot);
%     dJtest = Jp2-Jp

  %given Jdot, can compute qddot:
  % pdot = J*qdot
  % pddot = Jdot*qdot + J*qddot
  % qddot = J_inv*(pddot - Jdot*qdot)
  qddotOld = qddot;

  qddot = inv(Jp)*(pvecDdot-JdotNom*q_s*sdot^2); %compute d^2 q/dt^2 at s, sdot, sddot
  qddotvecs = [qddotvecs,qddot]; %save for plotting
 
  %test: should have qddot*dt = qdot(t+1)-qdot(t)
  %WORKS
%   dqDot = qvecDot-qvecDotOld 
%   qddotDt = qddotOld*dt
    
    %pause;
    
end
figure(1)
plot(phivec,qvecs(1,:),'b',phivec,qvecs(2,:),'r',phivec,qvecs(3,:),'g')
xlabel('angle phi (rad)')
ylabel('joint angles (rad)')
title('inverse kin: q1(b), q2(r), q3(g)')

figure(2)
plot(phivec,qdotvecs(1,:),'b',phivec,qdotvecs(2,:),'r',phivec,qdotvecs(3,:),'g')
xlabel('angle phi (rad)')
ylabel('joint velocities (rad/sec)')
title('q1Dot(b), q2Dot(r), q3Dot(g)')

figure(3)
plot(phivec,qddotvecs(1,:),'b',phivec,qddotvecs(2,:),'r',phivec,qddotvecs(3,:),'g')
xlabel('angle phi (rad)')
ylabel('joint accels (rad/(sec^2)')
title('q1Ddot(b), q2Ddot(r), q3Ddot(g)')

%compute the scale factors for constraints:
aq1dot = qdotMax(1)/max(qdotvecs(1,:));
aq2dot = qdotMax(2)/max(qdotvecs(2,:));
aq3dot = qdotMax(3)/max(qdotvecs(3,:));

aq1ddot = sqrt(qDdotMax(1)/max(qddotvecs(1,:)));
aq2ddot = sqrt(qDdotMax(2)/max(qddotvecs(2,:)));
aq3ddot = sqrt(qDdotMax(3)/max(qddotvecs(3,:)));

avec = [aq1dot,aq2dot,aq3dot,aq1ddot,aq2ddot,aq3ddot]

%compute max omega as multiplier of nominal omega=1.0
maxOmega = min(avec)

%%%%%%%%%%%%%%%%


