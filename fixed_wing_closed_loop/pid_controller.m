function [ax, ay, ah, pid_errors] = pid_controller(s, r, Ts, pid_errors)

% uav position and reference position
x = s(1); y = s(2); h = s(3);
rx = r(1); ry = r(2); rh = r(3);

% Errors
ex = rx - x; ey = ry - y; eh = rh - h;

% Integral Errors
pid_errors.ex_int = pid_errors.ex_int + Ts*ex;
pid_errors.ey_int = pid_errors.ey_int + Ts*ey;
pid_errors.eh_int = pid_errors.eh_int + Ts*eh;

% Derivative Errors
dex = (ex - pid_errors.ex_prev)/Ts;
dey = (ey - pid_errors.ey_prev)/Ts;
deh = (eh - pid_errors.eh_prev)/Ts;

% PID Gains
Kp_x = 0.5; Ki_x = 0.1; Kd_x = 1;
Kp_y = 0.5; Ki_y = 0.1; Kd_y = 1;
Kp_h = 0.3; Ki_h = 0.1; Kd_h = 0.8;

% PID Outputs (accelerations)
ax = Kp_x*ex + Ki_x*pid_errors.ex_int + Kd_x*dex;
ay = Kp_y*ey + Ki_y*pid_errors.ey_int + Kd_y*dey;
ah = Kp_h*eh + Ki_h*pid_errors.eh_int + Kd_h*deh;

% Update previous errors for next step
pid_errors.ex_prev = ex;
pid_errors.ey_prev = ey;
pid_errors.eh_prev = eh;

end