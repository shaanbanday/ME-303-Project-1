% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 11th March, 2025

clc; % Clear command window and workspace

% Table 1 Parameters
m = 1400; % Vehicle mass (kg)
a = 1.14; % Distance from centre of mass to the front axle (m)
b = 1.33; % Distance from centre of mass to the rear axle (m)
Iz = 2420; % Yaw inertia (kg·m^2)
dt = 0.01; % Δt (grid spacing)
t = 0:dt:5; % Simulate for 5 seconds

% Vehicle speed (100 km/h in m/s)
u = 100 / 3.6;

% Define different steering angles to test
delta_vals = [0.05, 0.1, 0.25];  % Three different delta values
labels = {'\delta = 0.05', '\delta = 0.1', '\delta = 0.25'};

figure; 
hold on; 
grid on; % Open a new figure and hold for multiple plots

for d = 1:length(delta_vals)
    delta = delta_vals(d);

    % Determine tire cornering stiffness based on delta
    if abs(delta) <= 0.06
        Cf = 20000;
        Cr = 20000;
    elseif abs(delta) <= 0.2
        Cf = 100;
        Cr = 100;
    else
        Cf = 0;
        Cr = 0;
    end

    % System of ODEs
    A = [- (Cf + Cr) / (m * u), - (a * Cf - b * Cr) / (m * u) - u;
         - (a * Cf - b * Cr) / (Iz * u), - (a^2 * Cf + b^2 * Cr) / (Iz * u)];
    B = [Cf / m; a * Cf / Iz];

    % Initial conditions
    x0 = [0; 0];  % [y_dot; psi_dot]
    psi0 = 0;     % Initial heading angle
    X0 = 0;       % Initial X position
    Y0 = 0;       % Initial Y position

    % Pre-allocate storage for states
    x_rk4 = zeros(2, length(t)); x_rk4(:,1) = x0;
    psi = zeros(1, length(t)); psi(1) = psi0;
    X = zeros(1, length(t)); X(1) = X0;
    Y = zeros(1, length(t)); Y(1) = Y0;

    % RK4 Solver
    for i = 1:length(t)-1
        k1 = A * x_rk4(:,i) + B * delta;
        k2 = A * (x_rk4(:,i) + 0.5 * dt * k1) + B * delta;
        k3 = A * (x_rk4(:,i) + 0.5 * dt * k2) + B * delta;
        k4 = A * (x_rk4(:,i) + dt * k3) + B * delta;
        x_rk4(:,i+1) = x_rk4(:,i) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

        % Integrate yaw rate to get heading angle
        psi(i+1) = psi(i) + dt * x_rk4(2,i);

        % Integrate velocity equations to get X and Y positions
        v_y = x_rk4(1,i) + a * x_rk4(2,i);
        X(i+1) = X(i) + dt * (u * cos(psi(i)) - v_y * sin(psi(i)));
        Y(i+1) = Y(i) + dt * (v_y * cos(psi(i)) + u * sin(psi(i)));
    end

    plot(X, Y, 'LineWidth', 2, 'DisplayName', labels{d});
end

% Final plot formatting
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Trajectory for Different Steering Angles on Icy Road');
legend('show');  % Show legend with delta values
axis equal; % Keep x and y axes equal when zooming in and out
