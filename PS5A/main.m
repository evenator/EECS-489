%Edward Venator
%EECS 489 Spring 2012
%Assignment 5a

clear all;
clc;

DH = [
    0   0.5 pi/2
    5   0   0
    4   0   0   ];

n_points = 100;

r = .1;
theta = zeros(n_points, 3);
p = zeros(3, n_points);
phi = zeros(1, n_points);

%Joint motion limitations
theta_dot_limits = [2, 2, .5];
theta_ddot_limits = [10, 10, 5];

%Find start position
p(:,1) = [4; 1; 1+r];
theta(1,:) = find_p(DH, p(:,1), [0 0 0], .0001);
phi(1) = 0;

%Create path with n points
for i=2:n_points
    phi(i) = 2*pi * (i-1)/n_points;
    p(:,i) = [4;
        1 + r * sin(2*pi * (i-1)/n_points);
        1 + r * cos(2*pi * (i-1)/n_points)];
    theta(i,:) = find_p(DH, p(:,i), theta(i-1,:), .000001);
end

figure(1)
plot(p(2,:),p(3,:));
title('Circular path in y-z plane');


figure(2);
plot(phi,theta(:,1),'b',phi,theta(:,2),'r',phi,theta(:,3),'g');
title('Joint Angles 1(b), 2(r), 3(g)');

%For each point, find the velocity and acceleration, using a test value of omega
omega = 72 ;
T = 2 * pi / (omega*n_points);

theta_dot = zeros(n_points, 3);
theta_ddot = zeros(n_points, 3);
for i = 1:n_points;
    if(i<n_points)
        if(i==1)
            theta_ddot(i,:) = (theta(i+1,:)-2*theta(i,:)+theta(n_points,:)) / T;
        else
            theta_ddot(i,:) = (theta(i+1,:)-2*theta(i,:)+theta(i-1,:)) / T;
        end
        theta_dot(i,:) = (theta(i+1,:)-theta(i,:)) / T;
    else
        theta_dot(i,:) = (theta(1,:)-theta(i,:)) / T;
        theta_ddot(i,:) = (theta(1,:)-2*theta(i,:)+theta(i-1,:)) / T;
    end
end

%Plot Velocities
figure(3);
plot(phi,theta_dot(:,1),'b',phi,theta_dot(:,2),'r',phi,theta_dot(:,3),'g');
title('Joint Velocities 1(b), 2(r), 3(g)');

%Plot Accelerations
figure(4);
plot(phi,theta_ddot(:,1),'b',phi,theta_ddot(:,2),'r',phi,theta_ddot(:,3),'g');
title('Joint Acclerations 1(b), 2(r), 3(g)');

%Determine velocity bounds
theta_dot_max = max(theta_dot,[],1);
theta_dot_max = theta_dot_max .* (theta_dot_max > 0);
theta_dot_min = min(theta_dot,[],1);
theta_dot_min = abs(theta_dot_min .* (theta_dot_min < 0));
theta_dot_bounds = max([theta_dot_min;theta_dot_max],[],1);

%Determine acceleration bounds
theta_ddot_max = max(theta_ddot,[],1);
theta_ddot_max = theta_ddot_max .* (theta_ddot_max > 0);
theta_ddot_min = min(theta_ddot,[],1);
theta_ddot_min = abs(theta_ddot_min .* (theta_ddot_min < 0));
theta_ddot_bounds = max([theta_ddot_min;theta_ddot_max],[],1);

%Find out what to multiply the test omega by for the bounds to hit the
%limits
multiplier = min([ (theta_dot_limits ./ theta_dot_bounds) , (theta_ddot_limits ./ theta_ddot_bounds)])