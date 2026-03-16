function [C_Vx, C_Vy, C_Vh] = mpc_contractive_constraints(cs0, ref, CS, alpha)

% current errors at tk
ex0 = cs0(1) - ref(1,1);
ey0 = cs0(2) - ref(2,1);
eh0 = cs0(3) - ref(3,1);
edx0 = cs0(4) - ref(4,1);
edy0 = cs0(5) - ref(5,1);
edh0 = cs0(6) - ref(6,1);

% terminal errors at tk+Hp
exT = CS(1,end) - ref(1,end);
eyT = CS(2,end) - ref(2,end);
ehT = CS(3,end) - ref(3,end);
edxT = CS(4,end) - ref(4,end);
edyT = CS(5,end) - ref(5,end);
edhT = CS(6,end) - ref(6,end);

% current Lyapunov values
Vx0 = 0.5*(ex0^2 + edx0^2);
Vy0 = 0.5*(ey0^2 + edy0^2);
Vh0 = 0.5*(eh0^2 + edh0^2);

% terminal Lyapunov values
VxT = 0.5*(exT^2 + edxT^2);
VyT = 0.5*(eyT^2 + edyT^2);
VhT = 0.5*(ehT^2 + edhT^2);

% contractive residuals
C_Vx = VxT - alpha*Vx0;
C_Vy = VyT - alpha*Vy0;
C_Vh = VhT - alpha*Vh0;

end