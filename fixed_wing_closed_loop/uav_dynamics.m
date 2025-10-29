function dsdt = uav_dynamics(t, s, params)
    
    % state extraction
    h = s(3); Vg = s(4); gamma = s(5); psi = s(6);

    % parameters extraction
    m = params.m;
    ga = params.ga;
    Vmw = params.Vmw;
    Kn = params.Kn;

    % gust model (wind disturbances)
    Vw_normal = 0.215 * Vmw * log10(h);
    Vw_tan = 0.09 * Vmw * randn;
    Vw = Vw_normal + Vw_tan;

    % reference state
    s_ref = ref_state_circle(t);

    % Controller
    [ax, ay, ah, ~] = sf_controller(t, s, s_ref);

    % double integrator mapping 
    [phib, ng, Th, Dg] = ... 
         di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params);

    % Lift
    Lf = ng*m*ga;

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
