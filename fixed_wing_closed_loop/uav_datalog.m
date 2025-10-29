function logs = uav_datalog(t, S, params, ref_fun, ctrl_fun)

    % parameters extraction
    Vmw = params.Vmw;
    m = params.m;
    ga = params.ga;

    % states
    logs.x = S(:,1);
    logs.y = S(:,2);
    logs.h = S(:,3);
    logs.Vg = S(:,4);
    logs.gamma = S(:,5);
    logs.psi = S(:,6);

    % prealloc
    N = numel(t);
    logs.ref = zeros(N,9);
    logs.E = zeros(N,3);
    logs.U = zeros(N,3);
    logs.Vw = zeros(N,1);
    logs.Th = zeros(N,1);
    logs.ng = zeros(N,1);
    logs.phib = zeros(N,1);
    logs.Dg = zeros(N,1); 
    logs.Lf = zeros(N,1);

    % loop
    for i = 1:N
        % state, reference, error & control log
        s = S(i,:).';
        r = ref_fun(t(i));
        [ax, ay, ah] = ctrl_fun(t(i), s, r);
        logs.ref(i,:) = r.';
        logs.E(i,:) = (s(1:3) - r(1:3)).';
        logs.U(i,:) = [ax, ay, ah];

        % wind log
        h = s(3);
        Vw_normal = 0.215 * Vmw * log10(h);
        Vw_tan = 0.09 * Vmw * rand;
        Vw = Vw_normal + Vw_tan;
        logs.Vw(i) = Vw;

        % phib, Th, Dg & Lf log
        Vg = s(4); gamma = s(5); psi = s(6);
        [phib, ng, Th, Dg] = ... 
        di_mapping(ax, ay, ah, psi, gamma, Vg, Vw, params);
        logs.phib(i) = phib;
        logs.ng(i) = ng;
        logs.Th(i) = Th;
        logs.Dg(i) = Dg;
        logs.Lf(i) = ng*m*ga;
    end
end
