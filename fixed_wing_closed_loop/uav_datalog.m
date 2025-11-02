function logs = uav_datalog(t_all, S_all, tk, U_d, ref_fun, params)

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

% discrete controller outputs
logs.U_d = U_d;

% countinuous controller outputs (zero order hold)
U = interp1(tk(1:end-1), U_d, t_all, 'previous', 'extrap');
logs.U = U;

% continuous Reference log (r(t))
N = length(t_all);
ref_cont = zeros(N,3);
 for i = 1:N
    r = ref_fun(t_all(i));
    ref_cont(i,:) = r(1:3).';
 end
logs.ref_cont = ref_cont;

% sampled Reference log (r(tk))
K = length(tk);
ref_samp = zeros(K,3);
 for k = 1:K
    r = ref_fun(tk(k));
    ref_samp(k,:) = r(1:3).';
 end
logs.ref_samp = ref_samp;

% countinuous errors log
E = [x - ref_cont(:,1), y - ref_cont(:,2), h - ref_cont(:,3)];
logs.E = E;

% Th, ng and phb mapping
Th  = zeros(N,1);
ng  = zeros(N,1);
phb = zeros(N,1);
Vw = zeros(N,1);
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

end