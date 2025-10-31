function logs = uav_datalog(t_all, S_all, tk, U_d, ref_fun, params)
% Outputs (struct logs)
%   .U.hold        (Nx3)         : U_d απλωμένο (ZOH) στα σημεία t_all
%   .map.(Th, ng, phib)          : mapped φυσικά σήματα σε t_all

% State log (continuous)
logs.state.x = S_all(:,1); 
logs.state.y = S_all(:,2);
logs.state.h = S_all(:,3);
logs.state.Vg = S_all(:,4);
logs.state.gamma = S_all(:,5);
logs.state.psi = S_all(:,6);

% velocities log
logs.vel.xdot = Vg .* cos(gamma) .* cos(psi);
logs.vel.ydot = Vg .* cos(gamma) .* sin(psi);
logs.vel.hdot = Vg .* sin(gamma);

% Continuous Reference log
N = numel(t_all);
ref_cont = zeros(N,3);
for i = 1:N
    r = ref_fun(t_all(i));
    ref_cont(i,:) = r(1:3).';
end
logs.ref.cont = ref_cont;

% Sampled Reference log
K = numel(tk);
ref_samp = zeros(K,3);
for k = 1:K
    r = ref_fun(tk(k));
    ref_samp(k,:) = r(1:3).';
end
logs.ref.samp = ref_samp;

% countinuous errors log
E = [ x - ref_cont(:,1),  y - ref_cont(:,2),  h - ref_cont(:,3) ];
logs.err = E;

% discrete controller outputs
logs.U.discrete = U_d;

% ---------- Mapping φυσικών σημάτων (continuous lattice) ----------
% Χρησιμοποιούμε το U_hold (ZOH) και τα τρέχοντα states για di_mapping
Th  = zeros(N,1);
ng  = zeros(N,1);
phb = zeros(N,1);

% Αν θες και άνεμο, φτιάξε εδώ τον Vw(t), π.χ. μηδενικός:
Vw = zeros(N,1);

for i = 1:N
    ax = U_hold(i,1);  ay = U_hold(i,2);  ah = U_hold(i,3);
    [phb(i), ng(i), Th(i), ~] = di_mapping(ax, ay, ah, ...
                              psi(i), gamma(i), Vg(i), Vw(i), params);
end
logs.map.Th   = Th;
logs.map.ng   = ng;
logs.map.phib = phb;

end

