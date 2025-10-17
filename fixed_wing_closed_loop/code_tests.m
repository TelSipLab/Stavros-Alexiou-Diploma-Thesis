% Define time vector
tt = linspace(0, 100, 1000);  % time from 0 to 100 seconds

% Initialize position array
circle = zeros(length(tt), 3);

% Generate reference trajectory
for i = 1:length(tt)
    ref = ref_state_circle(tt(i));  % Call function at each time
    circle(i, :) = [ref(1), ref(2), ref(3)];  % x, y, h
end

% Plot the reference circle
figure;
plot3(circle(:,1), circle(:,2), circle(:,3), 'r--', 'LineWidth', 1.5); 
xlabel('x [m]');
ylabel('y [m]');
zlabel('h [m]');
title('Reference Circle');
grid on;
axis equal;
