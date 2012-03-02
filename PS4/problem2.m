%Edward Venator
%EECS 489 Spring 2012
%Problem Set 4
%Problem 2

%Begin at the home position
%Crudely simulate arm dynamics with the springs described in the problem
%When the arm comes to rest (net torques all below a threshold), end
%simulation

%Gravity
g = [-9.8 0 0];

%Denavit-Hartenberg Parameters
DH = [
    0 pi/2 .5
    0 -pi/2 0
    0 pi/2 1
    0 -pi/2 0
    0 pi/2 1
    0 -pi/2 0
    0 0 1
];

payload_offs = [
    0
    .2
    0
];

%Add payload to DH table as an eighth link
DH(8,:) = [.2 0 0];
%Also need to add pi/2 to theta 7

%Masses
m = [ 
    100
    0
    50
    0
    25
    0
    0
    .2 ];

%Torque threshold to end simulation
threshold = .0001;

%Start at home
theta = zeros(8,1);
[~, p, z] = tooltransform(theta_a, DH);

net_torque = joint_torques(theta, p, z, m, g);

while sum(abs(net_torque)>threshold)
    %Simulate a step
    theta_dot = net_torque/100000; %Assumes all links have .0001 rotational inertia and critical damping
    theta(1:7) = theta(1:7) + theta_dot;
    %Calculate new torques
    net_torque = joint_torques(theta, p, z, m, g);
end

%Remove the pi/2 radian offset from the last link, which was created as a
%convenience
theta(7) = theta(7) - pi/2;

%Print results
final_theta = theta(1:7)