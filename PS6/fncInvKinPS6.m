function qvec = fncInvKinPS6(pvec,DH) 
%does inverse kinematics for 2DOF robot of PS6
%provide x,y, coords in pvec, and DH params in DH matrix
%return q1,q2 in qvec
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
C = acos(cosC); %rtns value in range 0 to pi, i.e. elbow-down
q2 = pi-C;
%phi is elevation angle from base to hand
phi = atan2(ydes,xdes);
%gamma = angle from link1 to rvec (vec from base to hand)

%b*b = a*a+rReach*rReach -2*a*rReach*cos(gamma)
cosGamma = (a*a+rReach*rReach-b*b)/(2*a*rReach);
gamma = acos(cosGamma);
q1 = phi-gamma;
qvec = [q1;q2];

end

