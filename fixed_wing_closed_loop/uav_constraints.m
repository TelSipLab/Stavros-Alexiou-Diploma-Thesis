function C = uav_constraints()

% controller constraints
C.Th_min = 0;
C.Th_max = 120;
C.ng_min = 0;
C.ng_max = 2;
C.phib_min = -30;
C.phib_max = 30;

% state constraints
C.Vg_min = 25;
C.Vg_max = 35;
C.gamma_max = 15;
C.gamma_min = -15;
C.psi_max = 15;
C.psi_min = -15;

end