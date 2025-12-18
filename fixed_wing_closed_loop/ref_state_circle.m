function s_ref = ref_state_circle(t)

    % horizontal circle with constant height
    C = [10; -10]; % circle center (x,y)
    z = 80; % height
    R = 150; % radius
    omega = 0.5; % angular speed

    % reference state
    x = C(1) + R*sin(omega*t);
    y = C(2) + R*cos(omega*t);
    h = z;
    dx = R*omega*cos(omega*t);
    dy = -R*omega*sin(omega*t);
    dh = 0;
    d2x = -R*(omega^2)*sin(omega*t);
    d2y = -R*(omega^2)*cos(omega*t);
    d2h = 0;

    % output
    s_ref = [x; y; h; dx; dy; dh; d2x; d2y; d2h];
end
