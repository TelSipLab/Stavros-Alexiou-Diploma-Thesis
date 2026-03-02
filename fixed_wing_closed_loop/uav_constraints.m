function C = uav_constraints()

%% real controller output constraints

% thrust
C.Th_min = 2;
C.Th_max = 120;

% g-load
C.ng_min = 0.1;
C.ng_max = 2;

% banking angle
C.phib_min = -30; 
C.phib_max = 30;

%% system state constraints

% ground speed
C.Vg_min = 25;
C.Vg_max = 35;

% flight path angle
C.gamma_max = 15;
C.gamma_min = -15;

% heading angle
C.psi_max = 15;
C.psi_min = -15;

end