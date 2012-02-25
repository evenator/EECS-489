%Edward Venator
%EECS 489 Spring 2012
%Assignment 2, Problem 1

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
    4.649228490139556e-001 4.298302356635038e+000 3.599334611113541e+000
    2.528280459777016e+000 6.175335696777363e+000 3.198201450386814e+000
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
    n1 = c3*(c12) - s3*(s12);
    n2 = c3*(s12) + s3*(c12);
    A(i,1) = n2/n1 * c1 - s1;
    A(i,2) = n2/n1 * c12 - s12;
    k(i) = n2/n1;
end
l = A \ k