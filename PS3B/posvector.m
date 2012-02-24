function [ vec ] = posvector( T )
%Extracts a vector of position coordinates and Euler ZYZ Angles from a
%Homogeneous Tranformation Matrix T
    vec(1:3) = T(1:3,4);
    vec(4) = atan2(T(2,3),T(1,3));
    vec(5) = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
    vec(6) = atan2(-T(3,2), -T(3,3));
end

