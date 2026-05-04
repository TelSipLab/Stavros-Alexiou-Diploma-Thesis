function [C_VBF, C_tracking, C_barrier] = mpc_contractive_constraint(cs0, ref, CS, alpha, obstacles, obst_params)

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

% current lyapunov function values
Vx0 = 0.5*(ex0^2 + edx0^2);
Vy0 = 0.5*(ey0^2 + edy0^2);
Vh0 = 0.5*(eh0^2 + edh0^2);
V0 = Vx0 + Vy0 + Vh0;

% terminal lyapunov function values
VxT = 0.5*(exT^2 + edxT^2);
VyT = 0.5*(eyT^2 + edyT^2);
VhT = 0.5*(ehT^2 + edhT^2);
VT = VxT + VyT + VhT;

% barrier function terms
BF0 = 0;
BFT = 0;
for m = 1:numel(obstacles)
    dim0 = norm(cs0(1:3) - obstacles(m).pos);
    dimT = norm(CS(1:3,end) - obstacles(m).pos);
    BF0 = BF0 + obst_params.Ko_BF * obstacle_barrier_potential(dim0, obst_params);
    BFT = BFT + obst_params.Ko_BF * obstacle_barrier_potential(dimT, obst_params);
end

% contractive constraint with barrier function
VBF0 = V0 + BF0;
VBFT = VT + BFT;
C_VBF = VBFT - alpha*VBF0;    % used constraint
C_tracking = VT - alpha*V0;   % used only for evaluation
C_barrier = BFT - alpha*BF0;  % ==||==

end

function Uo = obstacle_barrier_potential(dim, obst_params)

dc = obst_params.r_min;
rho0 = obst_params.d0_bar - dc;

if dim > dc && dim <= obst_params.d0_bar
    Uo = log(rho0^4 / (rho0^4 - (dim - obst_params.d0_bar)^4));
else
    Uo = 0;
end

end
