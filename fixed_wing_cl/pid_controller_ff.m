function [ax, ay, ah, pid_errors] = pid_controller_ff(cs, r, Ts, pid_gains, pid_errors)

% uav position and velocity
x = cs(1); y = cs(2); h = cs(3);
dx = cs(4); dy = cs(5); dh = cs(6);

% reference position, velocity and acceleration
rx = r(1); ry = r(2); rh = r(3);
rdx = r(4); rdy = r(5); rdh = r(6);
rd2x = r(7); rd2y = r(8); rd2h = r(9);

% position errors
ex = rx - x; ey = ry - y; eh = rh - h;

% integral errors
pid_errors.ex_int = pid_errors.ex_int + Ts*ex;
pid_errors.ey_int = pid_errors.ey_int + Ts*ey;
pid_errors.eh_int = pid_errors.eh_int + Ts*eh;

% derivative errors (finite difference - original PID)
dex = (ex - pid_errors.ex_prev)/Ts;
dey = (ey - pid_errors.ey_prev)/Ts;
deh = (eh - pid_errors.eh_prev)/Ts;

% Alternative derivative errors using velocity tracking.
% dex = rdx - dx;
% dey = rdy - dy;
% deh = rdh - dh;

% pid gains extraction
Kp_x = pid_gains(1); Ki_x = pid_gains(4); Kd_x = pid_gains(7);
Kp_y = pid_gains(2); Ki_y = pid_gains(5); Kd_y = pid_gains(8);
Kp_h = pid_gains(3); Ki_h = pid_gains(6); Kd_h = pid_gains(9);

% PID Outputs (accelerations) with reference acceleration feedforward
ax = rd2x + Kp_x*ex + Ki_x*pid_errors.ex_int + Kd_x*dex;
ay = rd2y + Kp_y*ey + Ki_y*pid_errors.ey_int + Kd_y*dey;
ah = rd2h + Kp_h*eh + Ki_h*pid_errors.eh_int + Kd_h*deh;

% Update previous errors for compatibility with the original PID structure
pid_errors.ex_prev = ex;
pid_errors.ey_prev = ey;
pid_errors.eh_prev = eh;

end
