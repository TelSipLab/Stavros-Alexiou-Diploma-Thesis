t_u = [0 2 4 6 8 10]; % Χρονικά σημεία
u_values = [0 1 0 -1 0 1]; % Αντίστοιχες τιμές u(t)
u_fun = @(t) interp1(t_u, u_values, t, 'linear', 'extrap');

function dx = double_integrator(t, x, u_fun)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = u_fun(t); % u αλλάζει με βάση το χρόνο
end

%% se diaforetiko script:

[t, x] = ode45(@(t, x) double_integrator(t, x, u_fun), tspan, x0);

