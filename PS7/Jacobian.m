function [ J ] = Jacobian( Amats ) 
%given the sequential Ai,i-1 matrices, compute the numerical values of the
%jacobian; this function assumes the robot is all revolute
[Nlinks,dummy]=size(Amats);
J=zeros(6,Nlinks);
z0 = [0;0;1];
zAxes = [z0];
frameOrigins = [0;0;0];

%make a matrix of successive z axes and origins from from 0 to frame Nlinks
for iLink=1:Nlinks
    Ai=Amats{iLink}; %A matrices are stored in "cells" of Amats; access using { } notation
    zi = Ai(1:3,3);
    Oi = Ai(1:3,4);
    zAxes=[zAxes,zi];
    frameOrigins=[frameOrigins,Oi];
end

rEndpt = frameOrigins(:,Nlinks+1); %frame 0 is index 1, so frame N is index N+1
for iLink=1:Nlinks
    zi = zAxes(:,iLink); %this is actually the i-1 z-axis, applicable to i'th column of J
    Oi = frameOrigins(:,iLink); %this is actually the i-1 origin, " " "
    Jpi = cross(zi,(rEndpt-Oi)); %generic formula for revolute jnt: uses cross product
    Jai = zi; %assumes robot is all revolute
    J(:,iLink)=[Jpi;Jai];
end


end

