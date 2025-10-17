function [phib, ng, Th, Dg] = di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params)

% parameters extraction
m = params.m;
ga = params.ga;
p = params.p;
Area = params.Area;
CD0 = params.CD0;
Kn = params.Kn;
Kd = params.Kd;

% banking angle (phib)
phib_num = ay*cos(psi) - ax*sin(psi);
phib_den = cos(gamma)*(ah + ga) - ...
    sin(gamma)*(ax*cos(psi) + ay*sin(psi));
phib = atan(phib_num/phib_den);

% g-load (ng)
ng_num = cos(gamma)*(ah+ga) - ... 
    sin(gamma)*(ax*cos(psi) + ay*sin(psi));
ng_den = ga*cos(phib);
ng = ng_num/ng_den;

% drag (xriazete gia ton ipologismo tou Th)
Dg = ((0.5 * p * ((Vg - Vw)^2) * Area * CD0) + ...
         (2 * Kd * (Kn^2) * (ng^2) * (m^2))) / ...
         (p * ((Vg - Vw)^2) * Area);

% Thrust (Th)
Th = (sin(gamma)*(ah+ga) + ... 
    cos(gamma)*(ax*cos(psi) + ay*sin(psi)))*m + Dg;

end