function dx = double_integrator(t, x, u)
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = u;
end
