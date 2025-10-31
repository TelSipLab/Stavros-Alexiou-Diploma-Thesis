function logs = uav_datalog(t_all, S_all, tk, U_d, ref_fun, params)
% Outputs (struct logs)
%   .U.hold
%   .map.(Th, ng, phib)

% State log (continuous)
x = S_all(:,1); 
y = S_all(:,2);
h = S_all(:,3);
Vg = S_all(:,4);
gamma = S_all(:,5);
psi = S_all(:,6);
logs.x = x; 
logs.y = y;
logs.h = h;
logs.Vg = Vg;
logs.gamma = gamma;
logs.psi = psi;

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
logs.E = E;

% discrete controller outputs
logs.U_d = U_d;

% countinuous controller outputs (zero order hold)
U = interp1(tk(1:end-1), U_d, t_all, 'previous', 'extrap');
logs.U = U;

% Mapping
Th  = zeros(N,1);
ng  = zeros(N,1);
phb = zeros(N,1);
Vw = zeros(N,1);

for i = 1:N
    ax = U(i,1);  ay = U(i,2);  ah = U(i,3);
    [phb(i), ng(i), Th(i), ~] = di_mapping(ax, ay, ah, ...
                              psi(i), gamma(i), Vg(i), Vw(i), params);
end
logs.map.Th = Th;
logs.map.ng = ng;
logs.map.phib = phb;

end