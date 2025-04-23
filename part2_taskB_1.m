% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 10th March, 2025

clc; % clear command window

% Table 1 Parameters
m = 1400; % Vehicle mass (kg)
a = 1.14; % Distance from centre of mass to the front axle (m)
b = 1.33;  % Distance from centre of mass to the rear axle (m)
Cf = 25000; % Front tire cornering stiffness (N/rad)
Cr = 21000; % Rear tire cornering stiffness (N/rad)
Iz = 2420; % Yaw inertia (kg·m^2)
dt = 0.01; % Δt (grid spacing)
t = 0:dt:5; % Simulate for 5 seconds
x0 = [0; 0]; % Initial conditions
delta = 0.1; % Step steering angle (rad)
speeds_kmh = [20, 50, 75, 100, 200, 300]; % Speeds to test (km/h)
speeds_ms = speeds_kmh / 3.6; % Convert to m/s


% Pre-allocate storage for plots
yaw_rate_data = zeros(length(speeds_ms), length(t)); % each row one speed
lat_accel_data = zeros(length(speeds_ms), length(t)); % same layout

for idx = 1:length(speeds_ms)
    % Current forward speed (m/s)
    u = speeds_ms(idx);

    % System of ODEs
    A = [- (Cf + Cr) / (m * u), - (a * Cf - b * Cr) / (m * u) - u;
     - (a * Cf - b * Cr) / (Iz * u), - (a^2 * Cf + b^2 * Cr) / (Iz * u)];
    % Define A as a 2x2 matrix, with elements indicated in project outline

    B = [Cf / m; a * Cf / Iz]; % Define B as a 2x1 matrix

    % RK4 solver for x(t) = [y_dot; psi_dot]
    x_rk4 = zeros(2, length(t));
    x_rk4(:,1) = x0; % set initial condition
    for i = 1:length(t)-1
        k1 = A*x_rk4(:,i) + B*delta;
        k2 = A*(x_rk4(:,i) + 0.5*dt*k1) + B*delta;
        k3 = A*(x_rk4(:,i) + 0.5*dt*k2) + B*delta;
        k4 = A*(x_rk4(:,i) + dt*k3)     + B*delta;
        x_rk4(:,i+1) = x_rk4(:,i) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
    % Same algorithm as before

    % Store yaw rate
    yaw_rate_data(idx,:) = x_rk4(2,:);

    % Compute lateral acceleration: a_y = y_ddot + u * psi_dot
    % where y_ddot is the first row of A*x + B*delta at each time step,
    % because x = [y_dot; psi_dot], so x_dot(1) = y_ddot
    for i = 1:length(t)
        x_now = x_rk4(:,i);
        xdot_now = A*x_now + B*delta; % [y_ddot; psi_ddot]
        y_ddot = xdot_now(1); % first component
        lat_accel_data(idx,i) = y_ddot + u*x_now(2);
    end
    % Algorithm above was refined with some minimal help of LLM
end

% Plot Yaw Rate vs Time for each speed
figure; % open new figure
hold on; % keep multiple plots
grid on; % enable grid for clarity

for idx = 1:length(speeds_ms) % Run loop to plot for each speed
    plot(t, yaw_rate_data(idx,:), 'LineWidth', 1.5, ...
         'DisplayName', [num2str(speeds_kmh(idx)),' km/h']);
end

legend('Location', 'Best', 'Interpreter', 'Latex'); % Add a legend
xlabel('Time (s)', 'Interpreter', 'Latex'); % x-axis label
ylabel('$\dot{\psi}$ (rad/s)', 'Interpreter', 'Latex'); % x-axis label
hold off;

% Plot Lateral Accel vs Time for each speed
figure; % open another figure
hold on; % keep multiple plots
grid on; % enable grid for clarity
for idx = 1:length(speeds_ms) % Run loop to plot each speed
    plot(t, lat_accel_data(idx,:), 'LineWidth', 1.5, ...
         'DisplayName', [num2str(speeds_kmh(idx)),' km/h']);
end
legend('Location', 'Best', 'Interpreter', 'Latex');
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('$a_y$ (m/s$^2$)', 'Interpreter', 'Latex'); % LLM assisted line
hold off;