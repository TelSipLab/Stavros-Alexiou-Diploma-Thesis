function logs = uav_datalog(t_all, S_all, tk, U_d, Vw_d, ref_fun, params, obstacles, obst_params)

N = length(t_all);
K = length(tk);
logs.tk = tk;

% state evolution
x = S_all(:,1); logs.x = x;
y = S_all(:,2); logs.y = y;
h = S_all(:,3); logs.h = h;
Vg = S_all(:,4); logs.Vg = Vg;
gamma = S_all(:,5); logs.gamma = gamma;
psi = S_all(:,6); logs.psi = psi;

% velocities evolution
logs.xdot = Vg .* cos(gamma) .* cos(psi);
logs.ydot = Vg .* cos(gamma) .* sin(psi);
logs.hdot = Vg .* sin(gamma);

% continuous reference log (r(t))
ref_cont = zeros(N,3);
ref_vel_cont = zeros(N,3);
ref_acc_cont = zeros(N,3);
 for i = 1:N
    r = ref_fun(t_all(i));
    ref_cont(i,:) = r(1:3).';
    ref_vel_cont(i,:) = r(4:6).';
    ref_acc_cont(i,:) = r(7:9).';
 end
logs.ref_cont = ref_cont;
logs.ref_vel_cont = ref_vel_cont;
logs.ref_acc_cont = ref_acc_cont;

% sampled reference log (r(tk))
ref_samp = zeros(K,3);
ref_acc_samp = zeros(K-1,3);
 for k = 1:K
    r = ref_fun(tk(k));
    ref_samp(k,:) = r(1:3).';
    if k < K
        ref_acc_samp(k,:) = r(7:9).';
    end
 end
logs.ref_samp = ref_samp;
logs.ref_acc_samp = ref_acc_samp;

% countinuous position errors log (e1)
e1 = [x - ref_cont(:,1), y - ref_cont(:,2), h - ref_cont(:,3)];
logs.E = e1;

% continuous velocity errors log (e2)
e2 = [logs.xdot - ref_vel_cont(:,1), ...
        logs.ydot - ref_vel_cont(:,2), ...
        logs.hdot - ref_vel_cont(:,3)];
logs.Edot = e2;

% Lyapunov functions per axis
Vx = 0.5*(e1(:,1).^2 + e2(:,1).^2); logs.Vx = Vx;
Vy = 0.5*(e1(:,2).^2 + e2(:,2).^2); logs.Vy = Vy;
Vh = 0.5*(e1(:,3).^2 + e2(:,3).^2); logs.Vh = Vh;
Ve = Vx + Vy + Vh;
logs.Ve = Ve;
BF = zeros(N,1);
if nargin >= 9
    obstacles_active_until = obst_params.obstacles_active_until;
    for i = 1:N
        if t_all(i) > obstacles_active_until
            continue;
        end
        for m = 1:numel(obstacles)
            dim = norm([x(i); y(i); h(i)] - obstacles(m).pos);
            BF(i) = BF(i) + obst_params.Ko_BF * obstacle_barrier_potential(dim, obst_params);
        end
    end
end
logs.Vb = BF;
logs.Vtotal = Ve + BF;

% discrete controller outputs log
logs.U_d = U_d;

% countinuous controller outputs (zero order hold)
U = interp1(tk(1:end-1), U_d, t_all, 'previous', 'extrap');
logs.U = U;

% discrete wind log
logs.Vw_d = Vw_d;

% countinuous wind log (zero order hold)
Vw = interp1(tk(1:end-1), Vw_d, t_all, 'previous', 'extrap');
logs.Vw = Vw;

% continuous reference dynamic inversion outputs
ref_phib = zeros(N,1);
ref_ng = zeros(N,1);
ref_Th = zeros(N,1);
for i = 1:N
    rdx = ref_vel_cont(i,1);
    rdy = ref_vel_cont(i,2);
    rdh = ref_vel_cont(i,3);
    rd2x = ref_acc_cont(i,1);
    rd2y = ref_acc_cont(i,2);
    rd2h = ref_acc_cont(i,3);

    rVg = sqrt(rdx^2 + rdy^2 + rdh^2);
    rGamma = asin(rdh / rVg);
    rPsi = atan2(rdy, rdx);
    [ref_phib(i), ref_ng(i), ref_Th(i), ~] = ...
        di_mapping(rd2x, rd2y, rd2h, rPsi, rGamma, rVg, Vw(i), params);
end
logs.ref_phib_cont = ref_phib;
logs.ref_ng_cont = ref_ng;
logs.ref_Th_cont = ref_Th;

% countinuous Th, ng and phb mapping
Th = zeros(N,1);
ng = zeros(N,1);
phb = zeros(N,1);
 for i = 1:N
    ax = U(i,1);  
    ay = U(i,2);  
    ah = U(i,3);
    [phb(i), ng(i), Th(i), ~] = ... 
        di_mapping(ax, ay, ah, psi(i), gamma(i), Vg(i), Vw(i), params);
 end
logs.Th = Th;
logs.ng = ng;
logs.phib = phb;

% discrete (sampled) mapping 
S_samp = interp1(t_all, S_all, tk, 'linear', 'extrap');
Vg_d = S_samp(:,4); gamma_d = S_samp(:,5); psi_d = S_samp(:,6);
Th_d = zeros(K-1,1);
ng_d = zeros(K-1,1);
phb_d = zeros(K-1,1);
for k = 1:K-1
    ax = U_d(k,1);
    ay = U_d(k,2);
    ah = U_d(k,3);
    [phb_d(k), ng_d(k), Th_d(k), ~] = ...
        di_mapping(ax, ay, ah, psi_d(k), gamma_d(k), Vg_d(k), Vw_d(k), params);
end
logs.Th_d = Th_d;
logs.ng_d = ng_d;
logs.phib_d = phb_d;

% extend
logs.Th_d_zoh = interp1(tk(1:end-1), Th_d, t_all, 'previous', 'extrap');
logs.ng_d_zoh = interp1(tk(1:end-1), ng_d, t_all, 'previous', 'extrap');
logs.phib_d_zoh = interp1(tk(1:end-1), phb_d, t_all, 'previous', 'extrap');

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
