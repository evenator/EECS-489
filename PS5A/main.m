%Edward Venator
%EECS 489 Spring 2012
%Assignment 5a

DH = [
    0   0.5 pi/2
    5   0   0
    4   0   0   ];

n_points = 200;

r = 3;
theta = zeros(n_points, 3);
p = zeros(3, n_points);

%Find start position
p(:,1) = [4; 1; 1+r];
theta(1,:) = find_p(DH, p(:,1), [0 0 0], .0001);

%Create path with n points
for i=2:n_points
    p(:,i) = [4;
        1 + r * sin(2*pi * (i-1)/n_points);
        1 + r * cos(2*pi * (i-1)/n_points)];
    theta(i,:) = find_p(DH, p(:,i), theta(i-1,:), .0001);
end

figure(1)
plot(p(2,:),p(3,:));
title('Circular path in y-z plane');


figure(2);
plot(1:n_points,theta(:,1),1:n_points,theta(:,2),1:n_points,theta(:,3));
title('Joint Angles');

%For each point, find the velocity, assuming 1 rotation/sec
omega = 2;
T = 2 * pi * omega/n_points;

theta_dot = zeros(n_points, 3);
for i = 1:n_points;
    if(i<n_points)
        theta_dot(i,:) = (theta(i+1,:)-theta(i,:)) / T;
    else
        theta_dot(i,:) = (theta(1,:)-theta(i,:)) / T;
    end
end

theta_dot_max = max(theta_dot,1)
theta_dot_min = min(theta_dot,1)

figure(3);
plot(1:n_points,theta_dot(:,1),1:n_points,theta_dot(:,2),1:n_points,theta_dot(:,3));
title('Joint Velocities');