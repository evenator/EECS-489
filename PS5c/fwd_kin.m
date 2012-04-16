function [ Aout, Amats ] = fwd_kin( DH ) 
%compute forward kinematics of a robot described by DH values
[Nlinks,dummy]=size(DH); %this many links
Aout = eye(4,4);
Amats=cell(Nlinks,1); %uses cell data structure to store an array of arrays
for iLink=1:Nlinks
Ai = [cos(DH(iLink,4)), -sin(DH(iLink,4))*cos(DH(iLink,3)), sin(DH(iLink,4))*sin(DH(iLink,3)),  DH(iLink,1)*cos(DH(iLink,4));
       sin(DH(iLink,4)), cos(DH(iLink,4))*cos(DH(iLink,3)),   -cos(DH(iLink,4))*sin(DH(iLink,3)),  DH(iLink,1)*sin(DH(iLink,4));
        0,                 sin(DH(iLink,3)),                       cos(DH(iLink,3)),                      DH(iLink,2);
        0,                     0,                                   0,                                       1];
Aout = Aout*Ai;
Amats{iLink}= Aout;  
end

