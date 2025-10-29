function plots_func(t, logs, K)
%% log extraction
% states
x = logs.x; y = logs.y; h = logs.h;
Vg = logs.Vg; gamma = logs.gamma; psi = logs.psi;

% reference
rx = logs.ref(:,1); ry = logs.ref(:,2); rh = logs.ref(:,3);

% kynematics
xdot = Vg .* cos(gamma) .* cos(psi);
ydot = Vg .* cos(gamma) .* sin(psi);
hdot = Vg .* sin(gamma);

% controller outputs
xddot = logs.U(:,1); yddot = logs.U(:,2); hddot = logs.U(:,3);

% Th, ng & phib
Th   = logs.Th;
ng   = logs.ng;
phib = logs.phib;

%% fig1: 3D Trajectory
figure;
plot3(x, y, h, 'b-', 'LineWidth', 1.6); hold on;
plot3(rx, ry, rh, 'r--', 'LineWidth', 1.4);
plot3(x(1), y(1), h(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
plot3(x(end), y(end), h(end), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
xlabel('x [m]'); ylabel('y [m]'); zlabel('h [m]');
title('UAV 3D Trajectory vs Reference'); grid on; axis equal;
legend('UAV','Reference','Start','End','Location','best');

%% fig2: State Evolution
figure;
subplot(6,1,1); plot(t,x,'b','LineWidth',1.4); grid on; ylabel('x [m]'); title('x');
subplot(6,1,2); plot(t,y,'b','LineWidth',1.4); grid on; ylabel('y [m]'); title('y');
subplot(6,1,3); plot(t,h,'b','LineWidth',1.4); grid on; ylabel('h [m]'); title('h');
subplot(6,1,4); plot(t,Vg,'c','LineWidth',1.4); grid on; ylabel('V_g [m/s]'); title('Ground Speed');
subplot(6,1,5); plot(t,rad2deg(gamma),'m','LineWidth',1.4); grid on; ylabel('\gamma [deg]'); title('Flight Path Angle');
subplot(6,1,6); plot(t,rad2deg(psi),'m','LineWidth',1.4);   grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); title('Heading');
sgtitle('UAV State Evolution');

%% fig3: Velocities from logs
figure;
subplot(3,1,1); plot(t, xdot,'LineWidth',1.4); grid on; ylabel('\it\_x [m/s]');   title('Velocities');
subplot(3,1,2); plot(t, ydot,'LineWidth',1.4); grid on; ylabel('\it\_y [m/s]');
subplot(3,1,3); plot(t, hdot,'LineWidth',1.4); grid on; ylabel('\it\_h [m/s]'); xlabel('Time [s]');

%% fig4: Accelerations (controller outputs)
figure;
subplot(3,1,1); plot(t, xddot, 'LineWidth',1.4); grid on; ylabel('\it\_x [m/s^2]'); title('Accelerations (controller outputs)');
subplot(3,1,2); plot(t, yddot, 'LineWidth',1.4); grid on; ylabel('\it\_y [m/s^2]');
subplot(3,1,3); plot(t, hddot, 'LineWidth',1.4); grid on; ylabel('\it\_h [m/s^2]'); xlabel('Time [s]');

%% fig5: Th, ng, phib
figure;
subplot(3,1,1); plot(t, Th, 'LineWidth',1.4); grid on; ylabel('T_h [N]'); title('Engine Thrust');
subplot(3,1,2); plot(t, ng, 'LineWidth',1.4); grid on; ylabel('n_g [-]'); title('Load factor');
subplot(3,1,3); plot(t, rad2deg(phib), 'LineWidth',1.4); grid on; ylabel('\phi_b [deg]'); xlabel('Time [s]'); title('Bank angle');

%% fig6: Closed-loop poles
k1 = K(1); k2 = K(2);
Acl = [0 1; -k1 -k2];
p = eig(Acl);

figure; hold on; grid on; box on;
plot(real(p), imag(p), 'x', 'MarkerSize',12, 'LineWidth',2);
xlabel('Re'); ylabel('Im'); title('Closed-loop poles');
xl = xlim; yl = ylim;
plot([xl(1) xl(2)], [0 0], 'k-'); % Re axis
plot([0 0], [yl(1) yl(2)], 'k-'); % Im axis
legend('Poles','Location','best');

end