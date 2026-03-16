function ref = ref_state_hippodrome(t)

% trajectory parameters
vc = 32;         % cruising speed [m/s]
Ts = 60;         % straight duration [s]
R = 500;         % turn radius parameter
z = 80;          % constant height
Tt = (pi*R)/vc;  % turn duration

% segment times
t1 = Ts;
t2 = t1 + Tt;
t3 = t2 + Ts;
t4 = t3 + Tt;

% periodic trajectory
t_mod = mod(t, t4);

% segment selection

if t_mod <= t1 % bottom straight

    theta = 0;
    dtheta = 0;
    x = vc * t_mod;
    y = 0;

elseif t_mod <= t2 % first turn
    
    tr = t_mod - t1;
    [s, ds, ~] = quintic_scaling(tr, Tt);
    theta = pi * s;
    dtheta = pi * ds;
    x = vc*Ts + vc*integral(@(tau) cos(pi*quintic_s(tau,Tt)),0,tr);
    y = vc*integral(@(tau) sin(pi*quintic_s(tau,Tt)),0,tr);

elseif t_mod <= t3 % top straight
    
    tr = t_mod - t2;
    theta = pi;
    dtheta = 0;
    xe = vc*Ts + vc*integral(@(tau) cos(pi*quintic_s(tau,Tt)),0,Tt);
    ye = vc*integral(@(tau) sin(pi*quintic_s(tau,Tt)),0,Tt);
    x = xe - vc*tr;
    y = ye;

else % second turn
    
    tr = t_mod - t3;
    [s, ds, ~] = quintic_scaling(tr, Tt);
    theta = pi + pi*s;
    dtheta = pi*ds;
    ye = vc*integral(@(tau) sin(pi*quintic_s(tau,Tt)),0,Tt);
    x = vc*integral(@(tau) cos(pi + pi*quintic_s(tau,Tt)),0,tr);
    y = ye + vc*integral(@(tau) sin(pi + pi*quintic_s(tau,Tt)),0,tr);

end

% height
h = z;

% velocities
dx = vc*cos(theta);
dy = vc*sin(theta);
dh = 0;

% accelerations
d2x = -vc*sin(theta)*dtheta;
d2y =  vc*cos(theta)*dtheta;
d2h = 0;

% reference vector
ref = [x; y; h; dx; dy; dh; d2x; d2y; d2h];

end

% helper functions

function [s, ds, dds] = quintic_scaling(t,T)
tau = t/T;
s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
ds  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
dds = (60*tau - 180*tau.^2 + 120*tau.^3)/T^2;
end

function s = quintic_s(t,T)
tau = t/T;
s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
end