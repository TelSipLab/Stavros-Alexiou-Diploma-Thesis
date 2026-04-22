function dsdt = uav_dynamics(~, ss, u_k, Vw, params)
    
    % state extraction
    Vg = ss(4); gamma = ss(5); psi = ss(6);

    % parameters extraction
    m = params.m;
    ga = params.ga;

    % controller output --> dynamics input
    ax = u_k(1);
    ay = u_k(2);
    ah = u_k(3);

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
