%Edward Venator
%EECS 489 Spring 2012
%Assignment 2, Problem 2

%The product A1 * A2 * A3 was solved with Mathematica
%From the resulting matrix, the n_hat vector and p vector were extracted
%This yields a system of linear equations of the form:
%0 - p_y = n_y/n_x * (1 - p_x)
%Where p_y is the y coordinate of the tool
% p_y = s1 * L1 + L2 * s12
%and p_x is the x coordinate of the tool
% p_x = c1 *L1 + L2 * c12
%and the point (1,0) is the location of the laser target. 
%Rearranged, this gives a solvable system of linear equations:
% 0 - (L1 * s1 + L2 * s12) = n_y/n_x * (1 - (c1 * L1 + L2 * c12))
% L1*(n_y/n_x*c1 - s1) + L2*(n_y/n_x*c12 - s12) = n_y/n_x
%which can be expressed as a matrix equation of the form:
% A*l = k
%where A is a matrix of coefficients, and k is a vector of constants
%This can be solved for the l vector:
%l = k \ A
clear all;

theta = [
    3.3600 3.3000 3.8000
    3.9600 4.0600 3.9400
    1.2000 5.0400 3.6600
    1.2400 2.5800 2.5400
    1.4800 1.7200 2.6400
    0.0600 5.8400 3.2800
    3.2600 2.9200 2.5200
    3.2800 2.1200 2.2800
    2.0800 6.2000 3.2800
    2.9800 4.7000 3.8400
    4.2000 5.0000 3.5600
    1.7200 1.7800 2.5800
    4.5000 1.1600 2.5800
    5.2400 1.0600 2.7000
    5.9800 4.5400 3.5400
    0.8400 5.3600 3.5200
    5.8600 3.5400 3.5000
    2.5000 2.7800 2.2800
    4.4000 1.6400 2.4400
    0.1800 5.9600 3.2600
    6.2600 4.9000 3.5000
    6.2200 3.0600 3.1200
    0.8000 2.9200 2.7400
    4.6600 4.2400 3.7600
    5.5000 4.7600 3.5200
    3.8800 1.7200 2.3800
    2.1200 5.2600 3.6600
    2.4200 0.5000 3.0000
    5.4400 2.7400 3.0400
    1.0800 0.0200 3.2400
    ];
k = zeros(size(theta,1),1);
A = zeros(size(theta,1),2);
for i=1:size(theta,1)
    c1 = cos(theta(i,1));
    c2 = cos(theta(i,2));
    c3 = cos(theta(i,3));
    s1 = sin(theta(i,1));
    s2 = sin(theta(i,2));
    s3 = sin(theta(i,3));
    s12 = sin(theta(i,1)+theta(i,2));
    c12 = cos(theta(i,1)+theta(i,2));
    n1 = c3*(c1*c2 - s1*s2) - s3*(c2*s1 + c1*s2);
    n2 = c3*(c2*s1 + c1*s2) + s3*(c1*c2 - s1*s2);
    A(i,1) = n2/n1 * c1 - s1;
    A(i,2) = n2/n1 * c12 - s12;
    k(i) = n2/n1;
end
l = A \ k