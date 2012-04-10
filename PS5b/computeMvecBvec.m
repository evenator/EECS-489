function [ Mvec, Bvec ] = computeMvecBvec( DH, R, s )
%Given a Denavit-Hartenberg table DH, Radius R, and path s, calculate the M
%and B vectors for all joints

%Calculate hand position, derivative, etc
[pvec, p_s, p_ss] = nomHand(s, R);

%Fill in angles in DH table
DH(:,4) = fncInvKinPS5(pvec, DH);

%Calculate Jacobian and Hessian from DH table. Hessian() is a modified
%version of JacobiansPS5 that does not require knowledge of q_dot
[Jp, h_ijk] = Hessian(DH);

%Mvec = q_s = Jp^-1 * p_s
q_s = (p_s' / Jp')'
Mvec = q_s

%Bvec = p_ss - (q_s)' h_ijk (q_s)
hess_sum = zeros(3,1);
for i = 1:3
            hess_sum (i) = q_s' * reshape(h_ijk(i,:,:),[3 3]) * q_s
end
Bvec = p_ss - hess_sum;