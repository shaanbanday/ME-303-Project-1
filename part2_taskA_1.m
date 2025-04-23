% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 9th March, 2025

clc; % clear command window

% Table 1 Parameters
m = 1400; % Vehicle mass (kg)
a = 1.14; % Distance from centre of mass to the front axle (m)
b = 1.33;  % Distance from centre of mass to the rear axle (m)
Cf = 25000; % Front tire cornering stiffness (N/rad)
Cr = 21000; % Rear tire cornering stiffness (N/rad)
Iz = 2420; % Yaw inertia (kg·m^2)
u = 75 / 3.6; % Velocity in x direction (converted from km/h to m/s)
dt = 0.01; % Δt (grid spacing)
t = 0:dt:5; % Simulate for 5 seconds
x0 = [0; 0]; % Initial conditions

delta = 0.1; % Step steering angle (rad)

% System of ODEs
A = [- (Cf + Cr) / (m * u), - (a * Cf - b * Cr) / (m * u) - u;
     - (a * Cf - b * Cr) / (Iz * u), - (a^2 * Cf + b^2 * Cr) / (Iz * u)];
% Define A as a 2x2 matrix, with elements indicated in the project outline

B = [Cf / m; a * Cf / Iz]; % Define B as a 2x1 matrix

% Forward Euler Method
x_euler = zeros(2, length(t)); % Initialize 2xt vector (top row is for 
% lateral, bottom is for yaw)
x_euler(:,1) = x0; % Set first column as ICs
for i = 1:length(t)-1 % Run Forward Euler
    x_euler(:,i+1) = x_euler(:,i) + dt * (A * x_euler(:,i) + B * delta);
end

% Runge-Kutta Method (RK4)
x_rk4 = zeros(2, length(t)); % Initialize 2xt vector (same as Euler)
x_rk4(:,1) = x0; % Set first column as ICs
for i = 1:length(t)-1 % Run Runge-Kutta
    k1 = A * x_rk4(:,i) + B * delta;
    k2 = A * (x_rk4(:,i) + 0.5 * dt * k1) + B * delta;
    k3 = A * (x_rk4(:,i) + 0.5 * dt * k2) + B * delta;
    k4 = A * (x_rk4(:,i) + dt * k3) + B * delta;
    x_rk4(:,i+1) = x_rk4(:,i) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end
% Algorithm above was refined with some minimal help of LLM

% Analytical Solution
x_ana = zeros(2, length(t)); % Initialize analytical solution vector
I2 = eye(2); % 2x2 identity matrix
for i = 1:length(t)
    eA = expm(A * t(i)); % Matrix exponential of A*t(i)
    x_ana(:,i) = eA * x0 + (A \ (eA - I2)) * B * delta;
end

% Plotting lateral velocity results
figure; % open a new figure window
grid on; % Turn on the grid for clarity
subplot(2,1,1); % put this plot at the top
plot(t, x_euler(1,:), 'r', 'LineWidth', 1.5); % Plot Euler results in red
hold on; % Use same graph to plot RK4 and Analytical
plot(t, x_rk4(1,:), 'b', 'LineWidth', 1.5); % Plot RK4 results in blue
plot(t, x_ana(1,:), 'k', 'LineWidth', 1.5); % Plot Analytical in black
legend('Euler', 'RK4', 'Analytical', 'Location', 'Best', ...
    'Interpreter', 'Latex'); % Add legend
xlabel('Time (s)', 'Interpreter', 'Latex'); % x-axis label
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'Latex'); % y-axis label

% Plotting yaw velocity results
subplot(2,1,2); % put this plot at the bottom
plot(t, x_euler(2,:), 'r', 'LineWidth', 1.5);  % Plot Euler results in red
hold on; % Use same graph to plot RK4 and Analytical
plot(t, x_rk4(2,:), 'b', 'LineWidth', 1.5);  % Plot RK4 results in blue
plot(t, x_ana(2,:), 'k', 'LineWidth', 1.5);  % Plot Analytical  in black
legend('Euler', 'RK4', 'Analytical', 'Location', 'Best', ...
    'Interpreter', 'Latex'); % Add legend
xlabel('Time (s)', 'Interpreter', 'Latex'); % x-axis label
ylabel('$\dot{\psi}$ (rad/s)', 'Interpreter', 'Latex'); % y-axis label

hold off; % release the plot hold