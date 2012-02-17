%Denavit-Hartenburg Homogeneous Tranform Matrix Calculator
%Edward Venator
%EECS 489 PS1

%All rev joint angles = 1 rad
Variable = 1;
Pi = pi;

%Adept 550
dh_list_a550 = [
    300	0	329.5	Variable
    250	pi	0	Variable
    0	0	0	0
    0	0	0	Variable
];

%Kawasaki JS10
dh_list_js10 = [
100	(3 * pi / 2)	0	Variable
650	0	0	Variable
0	(pi / 2)	0	Variable
0	(3 * pi / 2)	600	Variable
0	(pi / 2)	0	Variable
0	0	125	Variable
];

%Robotics Research K-2107hr
dh_list_rr = [
    0	(Pi / 2)	20	Variable
    5.6	(Pi / 2)	0	Variable
    4.2	(Pi / 2)	38	Variable
    3.5	(Pi / 2)	0	Variable
    1.9	(Pi / 2)	38	Variable
    1.75	(Pi / 2)	0	Variable
    0	0		4.5	Variable
];

%Pick a DH Table
dh_list = dh_list_rr;

%Initialize tranform matrix
A = eye(4);

%Perform transforms
for dh_param = dh_list'
    a = dh_param(1);
    d = dh_param(3);
    theta = dh_param(4);
    alpha = dh_param(2);
    
    A = A * [
        cos(theta), -sin(theta) * cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
        sin(theta), cos(theta) * cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,          sin(alpha), cos(alpha), a;
        0,          0,          0,          1];
end

%Extract origin position and z-axis from final transform
pos = A(1:3,4)
b_vec = A(1:3,3)