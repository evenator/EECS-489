function [ q,qdot ] = Simulate(H,DH,q,qdot,tau)
%Pass in a matrix inertia tensor, DH params (that include q in 4th col),
%velocity and joint torques, and this function will perform
%integration over 10ms  using 10 steps of euler integration at 1ms

q = DH(:,4);
dt=.001;
for t=1:10
    Q_grav= gravitytorques(DH);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Here's the onestep integration
    qdotdot=inv(H)*(tau-Q_grav);
    q=q+qdot*dt+.5*qdotdot*dt^2;
    qdot=qdot+qdotdot*dt;
    DH(:,4)=q;
    
end

