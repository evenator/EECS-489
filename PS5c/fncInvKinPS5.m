function qvec = fncInvKinPS5(pvec,DH) 
%does inverse kinematics for 3DOF robot of PS5
%provide x,y,z coords in pvec, and DH params in DH matrix
%return q1,q2,q3 in qvec
xdes=pvec(1);
ydes=pvec(2);
zdes=pvec(3);
q1 = atan2(ydes,xdes);
%compute elbow angle from reach from O1 to O3
dx = xdes;
dy = ydes;
dz = zdes-DH(1,2); %parameter d1
rsqd = dx*dx+dy*dy+dz*dz;
rReach = sqrt(rsqd);
%c^2 = a^2 + b^2 - 2*a*b*cos(C)
a = DH(2,1);
b = DH(3,1);
cosC = (a*a+b*b-rsqd)/(2*a*b);
C = acos(cosC); %rtns value in range 0 to pi, i.e. elbow-down
q3 = pi-C;
%phi is elevation angle from O1 to O3
rHoriz = sqrt(xdes*xdes + ydes*ydes);
phi = atan2(dz,rHoriz);
%gamma = angle from link2 to rvec (vec from O1 to O3)
%b*b = a*a+rReach*rReach -2*a*rReach*cos(gamma)
cosGamma = (a*a+rReach*rReach-b*b)/(2*a*rReach);
gamma = acos(cosGamma);
q2 = phi-gamma;
qvec = [q1;q2;q3];


end

