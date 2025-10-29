function [ax, ay, ah, K] = sf_controller(~, s, s_ref)

    % extract current UAV state x(t)
    x = s(1); y = s(2); h = s(3);
    Vg = s(4); gamma = s(5); psi = s(6);
    dx = Vg * cos(gamma) * cos(psi);
    dy = Vg * cos(gamma) * sin(psi);
    dh = Vg * sin(gamma);

    % extract reference state r(t)
    rx = s_ref(1); ry = s_ref(2); rh = s_ref(3);
    rdx = s_ref(4); rdy = s_ref(5); rdh = s_ref(6);
    rd2x = s_ref(7); rd2y = s_ref(8); rd2h = s_ref(9);

    % calculate error signals
    ex1 = x - rx; ex2 = dx - rdx;
    ey1 = y - ry; ey2 = dy - rdy;
    eh1 = h - rh; eh2 = dh - rdh;

    % dirrect pole placement
    % p1 = -4;
    % p2 = -4;
    % k1 = p1 + p2;
    % k2 = p1*p2;
    % K = [k1 k2];

    % pole placement using physical parameters
    Ts = 7; % settling time (sec)
    zeta = 1; % damping ratio
    wn = 4.04/(zeta*Ts); % wn using the +-2% criterion
    k1 = wn^2;
    k2 = 2*zeta*wn;
    K = [k1 k2];

    % calculate controller output
    ax = -K * [ex1; ex2] + rd2x;
    ay = -K * [ey1; ey2] + rd2y;
    ah = -K * [eh1; eh2] + rd2h;
end