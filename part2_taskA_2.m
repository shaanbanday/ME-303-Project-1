% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 10th March, 2025

clc; % clear command window

% Table 1 Parameters
m = 1400;   % Vehicle mass (kg)
a = 1.14;   % Distance from centre of mass to the front axle (m)
b = 1.33;   % Distance from centre of mass to the rear axle (m)
Cf = 25000; % Front tire cornering stiffness (N/rad)
Cr = 21000; % Rear tire cornering stiffness (N/rad)
Iz = 2420;  % Yaw inertia (kg·m^2)
u = 75 / 3.6; % Velocity in x direction (converted from km/h to m/s)
dt_values = [0.1, 0.05, 0.01, 0.001]; % Different time steps to test
x0 = [0; 0]; % Initial conditions

delta = 0.1; % Step steering angle (rad)

% System of ODEs
A = [- (Cf + Cr) / (m * u), - (a * Cf - b * Cr) / (m * u) - u;
     - (a * Cf - b * Cr) / (Iz * u), - (a^2 * Cf + b^2 * Cr) / (Iz * u)];
% Define A as a 2x2 matrix, with elements indicated in the project outline

B = [Cf / m; a * Cf / Iz]; % Define B as a 2x1 matrix

colors = ['r', 'g', 'b', 'k']; % Colors for different dt values

% Plot Grid Independence

figure; % open first figure window for euler
grid on; % Turn on the grid for clarity
hold on; % Plot different time steps on same graph
xlabel('Time (s)'); % x-axis label
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'Latex');


figure; % open second figure window for RK4
grid on; % Turn on the grid for clarity
hold on; % Plot different time steps on same graph
xlabel('Time (s)'); % x-axis label
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'Latex');


for idx = 1:length(dt_values) % loop to plot each time step
    dt = dt_values(idx);
    t = 0:dt:5; % Simulate for 5 seconds
    
    % Euler's Method
    x_euler = zeros(2, length(t)); % Initialize 2xt vector (top row is for
    % lateral, bottom is for yaw)
    x_euler(:,1) = x0; % Set first column as ICs
    
    for i = 1:length(t)-1 % Run Forward Euler
        x_euler(:,i+1) = x_euler(:,i) + dt*(A*x_euler(:,i) + B*delta);
    end
    % same exact code as used previously
    
    % RK4 Method
    x_rk4 = zeros(2, length(t)); % Initialize 2xt vector (top row is for
    % lateral, bottom is for yaw)
    x_rk4(:,1) = x0; % Set first column as ICs
    
    for i = 1:length(t)-1 % Run RK4
        k1 = A * x_rk4(:,i) + B * delta;
        k2 = A * (x_rk4(:,i) + 0.5 * dt * k1) + B * delta;
        k3 = A * (x_rk4(:,i) + 0.5 * dt * k2) + B * delta;
        k4 = A * (x_rk4(:,i) + dt * k3) + B * delta;
        x_rk4(:,i+1) = x_rk4(:,i) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
    % same exact code as used previously
    
    % Plot Euler results for each time step
    figure(1); % First figure
    plot(t, x_euler(1,:), colors(idx),'DisplayName',['dt = ',num2str(dt)]);
    
    % Plot RK4 results for each time step
    figure(2); % second figure
    plot(t, x_rk4(1,:), colors(idx),'DisplayName',['dt = ',num2str(dt)]);
end