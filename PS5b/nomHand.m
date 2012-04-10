function [pvec,p_s,p_ss]=nomHand(s,R)
    x = 4;
	y = 1+R*sin(s);
	z = 1+R*cos(s);
    pvec = [x;y;z];
    %speed at omega=1...or pdot = p_s*sdot
    xdot=0;
    ydot=R*cos(s);
    zdot= -R*sin(s);
    p_s=[xdot;ydot;zdot];   
    %pddot = p_s*sddot + p_ss*(sdot)^2
    x_ss = 0;
    y_ss = -R*sin(s);
    z_ss = -R*cos(s);
    p_ss=[x_ss;y_ss;z_ss];    