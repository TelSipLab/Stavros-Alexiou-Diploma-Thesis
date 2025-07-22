function dsdt = uav_dynamics(t, s, params, control_vars)
    
    % states   
    x = s(1);
    y = s(2);
    h = s(3);
    Vg = s(4);
    gamma = s(5);
    psi = s(6);

    % parameters
    m = params.m;
    ga = params.ga;
    Area = params.Area;
    CD0 = params.CD0;
    Kn = params.Kn;
    Kd = params.Kd;
    p = params.p;
    Vmw = params.Vmw;

    % control variables
    ng = control_vars(1);
    phib = control_vars(2);
    Th = control_vars(3);
    
    % gust model, lift and drag
    Vm = Vmw;
    Vw_normal = 0.215 * Vm * log10(h);
    Vw_tan = 0.09 * Vm * randn;
    Vw = Vw_normal + Vw_tan;

    Lf = ng * (ga - m);

    Dg = ((0.5 * p * ((Vg - Vw)^2) * Area * CD0) + ...
         (2 * Kd * (Kn^2) * (ng^2) * (m^2))) / ...
         (p * ((Vg - Vw)^2) * Area);

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
