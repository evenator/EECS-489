%Edward Venator
%EECS 489 Spring 2012
%Problem Set 4
%Problem 1

%Given the masses of several joints, computes the torque on each joint to
%hold a given set of joint angles.


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

%Part A
theta_a = zeros(8,1);
theta_a(7) = pi/2;
[~, p, z] = tooltransform(theta_a, DH);

%For each joint j, calculate the sum of torques due to all masses i
%There's a much more efficient way to do this, but this works
tau_a = zeros(7,1);
for j = 1:7
    for i = 1:8
        J = mass_jacobian(p, z, i);
        tau_a(j) = tau_a(j) + m(i) * g * J(:,j);
    end
end

tau_a


%Part B
theta_b = ones(8, 1);
theta_b(7) = 1 + pi/2;
[~, p, z] = tooltransform(theta_b, DH);

%For each joint j, calculate the sum of torques due to all masses i
tau_b = zeros(7,1);
for j = 1:7
    for i = 1:8
        J = mass_jacobian(p, z, i);
        tau_b(j) = tau_b(j) + m(i) * g * J(:,j);
    end
end

tau_b
