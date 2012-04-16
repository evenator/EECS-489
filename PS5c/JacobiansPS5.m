function [ Jp,Jdot,Hijk ] = JacobiansPS5(DH,qvecDot ) 
%given the sequential Ai,i-1 matrices, compute the numerical values of the
%jacobian; this function assumes the robot is all revolute
% also compute d/dt of the Jacobian, and its terms
% in the Hessian, Hijk
Hijk=zeros(3,3,3);  %Hessian is a tensor indexed by i,j,k
Jp=zeros(3,3);
Jdot=zeros(3,3);
L2=DH(2,1);
L3=DH(3,1);
d1=DH(1,2);
q1=DH(1,4);
q2=DH(2,4);
q3=DH(3,4);
qdot1 = qvecDot(1);
qdot2 = qvecDot(2);
qdot3 = qvecDot(3);

rHoriz = L2*cos(q2)+L3*cos(q2+q3);
xFwd = cos(q1)*rHoriz;
yFwd = sin(q1)*rHoriz;
zFwd = d1+L2*sin(q2)+L3*sin(q2+q3);


Jp(1,1) = -sin(q1)*rHoriz;
Jp(2,1) = cos(q1)*rHoriz;
Jp(3,1) = 0;

Jp(1,2) = cos(q1)*(-L2*sin(q2)-L3*sin(q2+q3));%dx/dq2:
Jp(2,2)= sin(q1)*(-L2*sin(q2)-L3*sin(q2+q3));%dy/dq2
Jp(3,2)= L2*cos(q2)+L3*cos(q2+q3); %dz/dq2

Jp(1,3)= cos(q1)*(-L3*sin(q2+q3));%dx/dq3:
Jp(2,3)= sin(q1)*(-L3*sin(q2+q3));%dx/dq3
Jp(3,3)= L3*cos(q2+q3);

rHorizDot = -L2*sin(q2)*qdot2-L3*sin(q2+q3)*(qdot2+qdot3);
%confirmed the above relative to the geometric Jacobian function
%Jp(1,1) = -sin(q1)*(L2*cos(q2)+L3*cos(q2+q3))
Jdot(1,1)= -cos(q1)*qdot1*rHoriz -sin(q1)*rHorizDot;
%Hessian terms: differentiate each term of J_i,j w/rt each q_k
Hijk(1,1,1)= -cos(q1)*rHoriz; %dJ(1,1)/dq1
Hijk(1,1,2)=  -sin(q1)*(-L2*sin(q2)-L3*sin(q2+q3));%dJ(1,1)/dq2
Hijk(1,1,3)=  -sin(q1)*(-L3*sin(q2+q3));%dJ(1,1,)/dq3

Jdot(2,1)= -sin(q1)*qdot1*rHoriz +cos(q1)*rHorizDot;
Hijk(2,1,1) = -sin(q1)*rHoriz;%dJ(2,1)/dq1
Hijk(2,1,2) = cos(q1)*(-L2*sin(q2)-L3*sin(q2+q3));%dJ(2,1)/dq2
Hijk(2,1,3) = cos(q1)*(-L3*sin(q2+q3));%dJ(2,1)/dq3

Jdot(3,1)=0;
Hijk(3,1,1)=0;
Hijk(3,1,2)=0;
Hijk(3,1,3)=0;

term2=(-L2*sin(q2)-L3*sin(q2+q3));
term2Dot = -L2*cos(q2)*qdot2-L3*cos(q2+q3)*(qdot2+qdot3);
%Jp(1,2) = cos(q1)*(-L2*sin(q2)-L3*sin(q2+q3))=cos(q1)*term2 ;%dx/dq2:
Jdot(1,2)= -sin(q1)*qdot1*term2 + cos(q1)*term2Dot;
Hijk(1,2,1)=-sin(q1)*term2;
Hijk(1,2,2)= cos(q1)*(-L2*cos(q2)-L3*cos(q2+q3));
Hijk(1,2,3)= cos(q1)*(-L3*cos(q2+q3));

%Jp(2,2)= sin(q1)*(-L2*sin(q2)-L3*sin(q2+q3));%dy/dq2
Jdot(2,2)= cos(q1)*qdot1*term2 + sin(q1)*term2Dot;
Hijk(2,2,1)=cos(q1)*term2;
Hijk(2,2,2)=sin(q1)*(-L2*cos(q2)-L3*cos(q2+q3));
Hijk(2,2,3)=sin(q1)*(-L3*cos(q2+q3));

%Jp(3,2)= L2*cos(q2)+L3*cos(q2+q3); %dz/dq2
Jdot(3,2)= -L2*sin(q2)*qdot2 -L3*sin(q2+q3)*(qdot2+qdot3);
Hijk(3,2,1)=0;
Hijk(3,2,2)=-L2*sin(q2) -L3*sin(q2+q3);
Hijk(3,2,3)=-L3*sin(q2+q3);

term3 = -L3*sin(q2+q3);
term3Dot = -L3*cos(q2+q3)*(qdot2+qdot3);
%Jp(1,3)= cos(q1)*(-L3*sin(q2+q3));%dx/dq3:
Jdot(1,3) = -sin(q1)*qdot1*term3 +cos(q1)*term3Dot;
Hijk(1,3,1)= -sin(q1)*term3;
Hijk(1,3,2)= cos(q1)*(-L3*cos(q2+q3));
Hijk(1,3,3)= cos(q1)*(-L3*cos(q2+q3));

%Jp(2,3)= sin(q1)*(-L3*sin(q2+q3));%dx/dq3
Jdot(2,3) = cos(q1)*qdot1*term3 + sin(q1)*term3Dot;
Hijk(2,3,1)=cos(q1)*term3;
Hijk(2,3,2)= sin(q1)*(-L3*cos(q2+q3));
Hijk(2,3,3)= sin(q1)*(-L3*cos(q2+q3));

%Jp(3,3)= L3*cos(q2+q3);
Jdot(3,3) = -L3*sin(q2+q3)*(qdot2+qdot3);
Hijk(3,3,1)=0;
Hijk(3,3,2)= -L3*sin(q2+q3);
Hijk(3,3,3)= -L3*sin(q2+q3);


end

