function dsdt = uav_dynamics(t, s, params, K)
    
    % state extraction
    h = s(3); Vg = s(4); gamma = s(5); psi = s(6);

    % parameters extraction
    m = params.m;
    ga = params.ga;
    Vmw = params.Vmw;

    % gust model (wind disturbances)
    Vw_normal = 0.215 * Vmw * log10(h);
    Vw_tan = 0.09 * Vmw * randn;
    Vw = Vw_normal + Vw_tan;

    % original control variables extraction
    %ng = control_vars(1);
    %phib = control_vars(2);
    %Th = control_vars(3);

    % double integrator control variables extraction
    % ax = control_vars(1);
    % ay = control_vars(2);
    % ah = control_vars(3);
    
    % compute di control variables using LQR controller
    [ax, ay, ah] = lqr_controller(t, s, K);

    % double integrator mapping ( 
    [ng, phib, Th, Dg] = ... 
         di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params);

    % lift
    Lf = ng * (ga - m);

    % Kinematics
    dx = Vg * cos(gamma) * cos(psi);
    dy = Vg * cos(gamma) * sin(psi);
    dh = Vg * sin(gamma);

    % Dynamics
    dVg = (Th - Dg)/m - ga*sin(gamma);
    dgamma = (ga/Vg) * (ng*cos(phib) - cos(gamma));
    dpsi = (Lf*sin(phib))/(m*Vg*cos(gamma));

    % Return ds/dt
    dsdt = [dx; dy; dh; dVg; dgamma; dpsi];
end
