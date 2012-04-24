function [pvec,pdot,pddot]=desHand(t,omega)
    x = 0.5*cos(omega*t);
	y = 0.4;
    z=0.0;
    pvec = [x;y;z];
    %speed at omega=1...or pdot = p_s*sdot
    xdot= -0.5*omega*sin(omega*t);
    ydot=0;
    zdot=0;
    pdot=[xdot;ydot;zdot];  
    xddot = -0.5*omega^2*cos(omega*t);
    yddot = 0.0;
    zddot=0.0;
    pddot = [xddot;yddot;zddot];
    

  