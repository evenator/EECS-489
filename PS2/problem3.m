%Edward Venator
%EECS 489 Spring 2012
%Assignment 2, Problem 3

%The product A1 * A2 * A3 was solved with Mathematica
%From the resulting matrix, the n_hat vector and p vector were extracted
%This yields a system of linear equations of the form:
%0 - p_y = n_y/n_x * (1 - p_x)
%Where p_y is the y coordinate of the tool
% p_y = L1 * (s1*c1_offset + c1*s1_offset) + L2 * (s12*c12_offset + c12 * s12_offset)
%and p_x is the x coordinate of the tool
% p_x = L1 * (c1*c1_offset - s1*s1_offset) + L2 * (c12*c12_offset - s12*s12_offset)
%and the point (1,0) is the location of the laser target.
%Unfortunately, this equation is nonlinear with respect to the theta offset
%values we wish to solve for. However, it is possible to turn this into a
%linear equation by solving for a set of intermediate variables. There are
%nine such intermediate variables:
% d = t123_offset * c1_offset
% e = c1_offset
% f = s123_offset * s1_offset
% g = s1_offset
% h = t123_offset * c12_offset
% i = t123_offset * s12_offset
% j = c12_offset
% k = s12_offset
% l = t123_offset
%where
% c1_offset and s1_offset are the cosine and sine of offset angle 1
% c12_offset and s12_offset are the cosine and sine of the sum of offset angles 1 and 2
% t123_offset is the tangent of the sum of offset angles 1, 2, and 3
%
%Since we have more than 9 calibration points, it is possible to solve
%the resulting system of linear equations:
% (-n_x * y + n_y * x)/c123_offset - c123 * t123_offset = s123
%which can be expressed as a matrix equation of the form:
% A*solution_vector = k
% where A is a matrix of coefficients, and solution_vector contains
% variables d through l. k is a constant equal to s123
%This can be solved for the solution_vector:
%solution_vector = k \ A
%From the solution vector, offset angles can be calculated:
% offset1 = acos(e)
% offset2 = acos(j) - offset1
% offset3 = atan(l) - offset1 - offset2
clear all;

theta = [
    0.935582084662831 0.464945697480611 3.423041764242886
    2.619802107158008 3.942941789155497 4.709053342135887
    5.878515442360697 2.045847867097161 2.779030547354564
    3.524583188120033 5.111451464165370 4.096868597334137
    2.157111990129419 0.798553626531776 3.249540042731869
    2.875173700317071 0.382695046727761 3.386787095731789
    4.343602918101078 0.901142994464319 3.015372779030698
    1.354376879584788 3.531103189829548 3.245153419890124
    4.041799324874798 2.547376870445077 2.268659770833816
    3.910460562755201 5.518016946885467 3.851798053191861
    4.510454840883607 2.668499606659538 2.366391304724546
    3.693156781160284 1.769374663025819 2.621387610560735
    4.670054306710624 2.357200641648681 2.446475856292719
    0.594076233735676 2.098972927909183 2.831536647177370
    1.987484460665142 0.345903087554289 3.474902628575549
    1.206558478400626 5.609312333541132 3.905322292772058
    4.221443901084579 5.581784471670855 3.787773242109359
    1.802369153910894 5.829405924921725 3.847541711006087
    1.411868747625556 6.258047343772113 3.649379949946922
    2.703501217049336 1.642400778787074 2.782077557679599
    5.241971609968998 2.464733260932763 2.616981327892641
    3.251168656193447 4.897775035516864 4.233336481316632
    5.326666947972201 2.678534278469691 2.727357921395607
    0.707184074866703 1.867504694919344 2.892496751742639
    5.075937266961552 6.086469652779611 3.504354275243208
    3.670474012952479 6.093763266219386 3.592922287704697
    3.852622337373180 6.407485287375491 3.420515347714634
    4.889976910017113 1.366258898409974 2.823846949120064
    3.346961706506059 4.222194238988075 4.560643253470599
    5.559779530808699 3.970654922062697 4.189191572490511
    ];

L1 = 4;
L2 = 5;

k = zeros(size(theta,1),1);
A = zeros(size(theta,1),9);
for i=1:size(theta,1)
    c1 = cos(theta(i,1));
    c2 = cos(theta(i,2));
    c3 = cos(theta(i,3));
    s1 = sin(theta(i,1));
    s2 = sin(theta(i,2));
    s3 = sin(theta(i,3));
    s12 = sin(theta(i,1)+theta(i,2));
    c12 = cos(theta(i,1)+theta(i,2));
    s123 = sin(theta(i,1)+theta(i,2)+theta(i,3));
    c123 = cos(theta(i,1)+theta(i,2)+theta(i,3));
    n1 = c3*(c1*c2 - s1*s2) - s3*(c2*s1 + c1*s2);
    n2 = c3*(c2*s1 + c1*s2) + s3*(c1*c2 - s1*s2);
    A(i,1) = s123 * L1 *s1 + c123 * L1 * c1;
    A(i,2) = -c123 * L1 * s1 + s123 * L1 * c1;
    A(i,3) = s123 * L1 * c1 - c123 * L1 * s1;
    A(i,4) = -c123 * L1 * c1 - s123 * L1 * s1;
    A(i,5) = s123 * L2 * s12 + c123 * L2 * c12;
    A(i,6) = s123 * L2 * c12 - c123 * L2 * s12;
    A(i,7) = -c123 * L2 * s12 + s123 * L2 * c12;
    A(i,8) = -c123 * L2 * c12 - s123 * L2 * s12;
    A(i,9) = -c123;
    k(i) = s123;
end
vector_soln = A \ k;
offset1 = acos(vector_soln(2))
offset2 = acos(vector_soln(7)) - offset1
offset3 = atan(vector_soln(9)) - offset1 - offset2
