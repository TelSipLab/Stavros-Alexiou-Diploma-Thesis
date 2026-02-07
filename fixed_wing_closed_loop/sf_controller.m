function [ax, ay, ah, Kxy, Kh] = sf_controller(cs, r)

    % extract current UAV di control state x(t)
    x = cs(1); y = cs(2); h = cs(3);
    dx = cs(4); dy = cs(5); dh = cs(6);

    % extract reference state r(t)
    rx = r(1); ry = r(2); rh = r(3);
    rdx = r(4); rdy = r(5); rdh = r(6);
    rd2x = r(7); rd2y = r(8); rd2h = r(9);

    % calculate error signals
    ex1 = x - rx; ex2 = dx - rdx;
    ey1 = y - ry; ey2 = dy - rdy;
    eh1 = h - rh; eh2 = dh - rdh;

    % pole placement using physical parameters (x-y axis)
    Ts = 10; % settling time (sec)
    zeta = 1; % damping ratio
    wn = 4.04/(zeta*Ts); % wn using the +-2% criterion
    k1 = wn^2;
    k2 = 2*zeta*wn;
    Kxy = [k1 k2];

    % pole placement using physical parameters (h axis)
    Ts = 15; % settling time (sec)
    zeta = 1; % damping ratio
    wn = 4.04/(zeta*Ts); % wn using the +-2% criterion
    k1 = wn^2;
    k2 = 2*zeta*wn;
    Kh = [k1 k2];

    % % dirrect pole placement (x-y axis)
    % p1 = -0.7;
    % p2 = -0.7;
    % k1 = p1*p2;
    % k2 = -(p1+p2);
    % Kxy = [k1 k2];

    % % dirrect pole placement (h axis)
    % p1 = -0.4;
    % p2 = -0.4;
    % k1 = p1*p2;
    % k2 = -(p1+p2);
    % Kh = [k1 k2];

    % calculate controller output
    ax = -Kxy * [ex1; ex2] + rd2x;
    ay = -Kxy * [ey1; ey2] + rd2y;
    ah = -Kh * [eh1; eh2] + rd2h;

end