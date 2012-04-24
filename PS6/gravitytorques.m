function [ Q_grav ] = gravitytorques(DH)
%GRAVITYTORQUES Summary of this function goes here
%   Detailed explanation goes here
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Newton method for finding gravity torques on joints
    [Nlinks,dummy]=size(DH); %this many links

   %matrix to hold the forces at hand and each joint:
   F=zeros(3,Nlinks+1);
   M=zeros(3,Nlinks+1); %and matrix to hold the moments at hand and each jnt

    payload = 2; %2Kg mass in gripper
    g=9.8; %gravity, m/s^2

    [ Aout, Amats ] = fwd_kin( DH );  %fwd kin provides all A matrices
    
    %project forces and moments to the end effector
    F(:,Nlinks+1)=[0;-payload*g;0]; %force at the hand
    M(:,Nlinks+1)=[0;0;0]; %moment on the hand
    
    for i=Nlinks:-1:1
        %forces on i = (forces on i+1) + (weight of i+1)
        F(:,i)=F(:,i+1); %but we modeled links as massless, so F_i+1 + F_i = 0 for statics
        %moments on i = r cross F (offset to origin of forces, forces)
        Adistal=Amats{i};
        if i>1
            Aproximal=Amats{i-1};
        else
            Aproximal = eye(4,4);
        end
        r=Adistal(1:3,4)-Aproximal(1:3,4); %vector from origin i to origin i+1
        M(:,i)= M(:,i+1)  -cross(r,F(:,i+1)); %work from hand to base to compute
          %moments at each joint; m_i = m_i+1- r_i+1,i x F_i+1
    end
    %strip off z-components of moment vectors to get required motor torques
Q_grav=transpose(M(3,1:Nlinks));