function [Mvec,Bvec] = computeMvecBvec(DH,R,s)
% given robot description (DH), path description (implicit from R) and "s",
% can compute Jp, Mvec, and Bvec, where:
% qddot = Mvec*sddot + Bvec*sdot^2
    [pvec,p_s,p_ss]=nomHand(s,R); %compute hand pose at s
    qvec = fncInvKinPS5(pvec,DH);  %inverse kinematics: p-> q
    DH(:,4)=qvec; %assign these solution values to q-column of DH
    [ Aout, Amats ] = fwd_kin( DH ); %compute all the A matrices
    J = Jacobian( Amats );
    Jp = J(1:3,:); %translational part of Jacobian, 3x3
    %compute terms qdot, Jdot, qddot at sdot=1, i.e. omega=1
    q_s = inv(Jp)*p_s; %dq/ds: qdot = q_s*sdot
     %compute J,Jdot, Hessian:  Jp computation is redundant w/ above
     %JdotNom is at nominal speed, sdot=1, i.e. omega=1:
     %don't actually use the Hessian here... but could
    [ JpTest,JdotNom,Hijk ] = JacobiansPS5(DH,q_s );
    
    % from qddot = J_inv[pdotNom*omega_dot + 
    %              pddotNom*omega^2 - (JdotNom*qdotNom)*omega^2   ]
    % = Mvec(s)*omega_dot + Bvec(s)*omega^2
    %-> Bvec(s) = J_inv(pddotNom- JdotNom*qdotNom)  
    % and Mvec = J_inv*pdotNom = q_s
    Mvec = q_s; %q_s = inv(Jp)*p_s;    
    Bvec = inv(Jp)*(p_ss-JdotNom*q_s);  %this is a bvector, weighting term on sdot^2
    %return Mvec (q_s) and Bvec, so can compute corresponding qdot, qddot
    %external to this fnc 