function [phib, ng, Th] = di_mapping(ax, ay, ah, psi, gamma, m, ga, Dg)

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

% Thrust (Th)
Th = (sin(gamma)*(ah+ga) + ... 
    cos(gamma)*(ax*cos(psi) + ay*sin(psi)))*m + Dg;

end