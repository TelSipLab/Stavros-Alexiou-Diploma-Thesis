function [ax, ay, ah] = lqr_controller(t, s, K)

    % extract current UAV state
    x = s(1); y = s(2); h = s(3);
    Vg = s(4); gamma = s(5); psi = s(6);

    % calculate angular velocities using kynematics eq.
    dx = Vg * cos(gamma) * cos(psi);
    dy = Vg * cos(gamma) * sin(psi);
    dh = Vg * sin(gamma);

    % virtual state
    s_virtual = [x; dx; y; dy; h; dh];

    % desired trajectory for the UAV
    s_ref = ref_state_circle(t);

    % control signal calculation (controller output)
    e = s_virtual - s_ref;
    u = -K * e;
    ax = u(1); ay = u(2); ah = u(3);
end
