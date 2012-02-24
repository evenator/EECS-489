%Edward Venator
%EECS 489 Spring 2012
%Assignment 3b

%Given Denavit-Hartenberg Parameters and a desired position, find joint
%   angles to achieve that position by using the Jacobian.
% Loads DH parameters into DH
% Loads desired position/orientation transform into T_des

%DH Table
DH = [
    -pi/2   0.0     0;
    -pi/2   0.0     0;
    pi/2    4.2     0;
    pi/2    4.2     0;
    pi/2    1.9     38;
    pi/2    1.75    0;
    0.0    0.0      4.5;
    ];

%Desired position
T_des = [
    -0.8415 -0.5403  0         3.0714
     0.2919 -0.4546  0.8415  -56.074
    -0.4546  0.7081  0.5403  -45.0033
     0       0       0         1
    ];

%Calculate Desired Position/Euler Angle Vector
p_des = posvector(T_des);

%Threshold for error
thresh = .01;
%"Spring" Constant
k = .001;

%Initialize theta to all zeros
theta = zeros(7,1);

%Get the initial tool position and Jacobian
[T, J] = tooltransform(theta, DH);

%Calculate the error vector e
e = p_des - posvector(T);

%As long as error exists, move the arm toward p_des
while(norm(e) > thresh)
    %Inverse Jacobian Stuff to get new thetas
    force = k * e;
    theta_dot = force / J';
    theta = theta + theta_dot';
    %Recalculate Transform and Jacobian
    [T, J] = tooltransform(theta, DH);
    old_norm = norm(e);
    %Recalculate Error
    e = p_des - posvector(T);
    if(norm(e)>old_norm)
        k = k/10;
    end
    norm(e)
end

%Print Desired Theta List
fid = fopen('solution.txt','w');
fprintf(fid,'%f ',theta);
fclose(fid);