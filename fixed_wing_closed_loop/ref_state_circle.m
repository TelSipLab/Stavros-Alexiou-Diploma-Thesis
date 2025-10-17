function s_ref = ref_state_circle(t)

    % horizontal circle with constant height
    C = [-100; 0]; % circle center (x,y) (meters)
    z = 100; % height (meters)
    R = 100; % radius (meters)
    omega = 0.1; % angular speed (rad/s)

    % reference state calculation
    x = C(1) + R*cos(omega*t);
    y = C(2) + R*sin(omega*t);
    h = z;
    dx = -R*omega*sin(omega*t);
    dy = R*omega*cos(omega*t);
    dh = 0;

    s_ref = [x; y; h; dx; dy; dh];
end