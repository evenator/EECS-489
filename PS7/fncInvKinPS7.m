function qvec = fncInvKinPS7(pvec,DH) 
%does inverse kinematics for 2DOF robot of PS7: planar arm
%provide x,y, coords in pvec, and DH params in DH matrix
%return q1,q2 in qvec--"lefty" solution
xdes=pvec(1);
ydes=pvec(2);

%compute elbow angle from reach from base to hand
dx = xdes;
dy = ydes;

rsqd = dx*dx+dy*dy;

rReach = sqrt(rsqd);
%c^2 = a^2 + b^2 - 2*a*b*cos(C)
a = DH(1,1);
b = DH(2,1);
cosC = (a*a+b*b-rsqd)/(2*a*b);
%elbow angle... (included angle of triangle--supplement of q2)
C = acos(cosC); %rtns value in range 0 to pi, 
q2 = -(pi-C); %negate for lefty solution

%phi is elevation angle from base to hand
phi = atan2(ydes,xdes);
%gamma = angle from link1 to rvec (vec from base to hand)

%b*b = a*a+rReach*rReach -2*a*rReach*cos(gamma)
cosGamma = (a*a+rReach*rReach-b*b)/(2*a*rReach);
gamma = acos(cosGamma);
q1 = phi+gamma; %lefty solution for q1
qvec = [q1;q2];

end

