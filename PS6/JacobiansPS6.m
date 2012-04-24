function [ Jp,Jdot ] = JacobiansPS5(DH,qvecDot ) 
%jacobian and jacobian_dot; this function assumes the robot is all revolute
Jp=zeros(2,2);
Jdot=zeros(2,2);
L1=DH(1,1);
L2=DH(2,1);
q1=DH(1,4);
q2=DH(2,4);

q1dot = qvecDot(1);
q2dot = qvecDot(2);

xFwd = L1*cos(q1)+L2*cos(q1+q2);
yFwd = L1*sin(q1)+L2*sin(q1+q2);


Jp(1,1) = -L1*sin(q1)-L2*sin(q1+q2);%dx/dq1:
Jp(1,2)= -L2*sin(q1+q2);%dx/dq2

Jp(2,1)= L1*cos(q1)+L2*cos(q1+q2);%dy/dq1
Jp(2,2)= L2*cos(q1+q2); %dy/dq2

%Jdot terms...
%e.g.: dJ11/dq1*q1dot + dJ11/dq2*q2dot:
Jdot(1,1)=(-L1*cos(q1)-L2*cos(q1+q2))*q1dot -L2*cos(q1+q2)*q2dot;
Jdot(1,2)= -L2*cos(q1+q2)*q1dot -L2*cos(q1+q2)*q2dot;
Jdot(2,1)= (-L1*sin(q1) -L2*sin(q1+q2))*q1dot -L2*sin(q1+q2)*q2dot;
Jdot(2,2)= -L2*sin(q1+q2)*q1dot -L2*sin(q1+q2)*q2dot;% 

end

