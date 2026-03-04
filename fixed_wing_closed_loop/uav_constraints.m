function C = uav_constraints()

%% real controller output constraints

% thrust [N]
C.Th_min = 4;
C.Th_max = 120;

% g-load [G]
C.ng_min = 0.1;
C.ng_max = 2;

% banking angle [rad]
C.phib_min = deg2rad(-30); 
C.phib_max = deg2rad(30);

%% system state constraints ???

% ground speed [m/s]
C.Vg_min = 25;
C.Vg_max = 35;

% flight path angle [rad]
C.gamma_max = deg2rad(15);
C.gamma_min = deg2rad(-15);

% heading angle [rad]
C.psi_max = deg2rad(15);
C.psi_min = deg2rad(-15);

end