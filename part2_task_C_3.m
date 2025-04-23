% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 10th March, 2025

clc; % clear command window

% Table 1 Parameters
m = 1400;   % Vehicle mass (kg)
a = 1.14; % Distance from centre of mass to the front axle (m)
b = 1.33;  % Distance from centre of mass to the rear axle (m)
Cf; % Front tire cornering stiffness (N/rad)
Cr; % Rear tire cornering stiffness (N/rad)
Iz = 2420;  % Yaw inertia (kg·m^2)
dt = 0.01; % Δt (grid spacing)
t = 0:dt:5; % Simulate for 5 seconds
x0 = [0; 0]; % Initial conditions
delta = 0.01; % Step steering input

% Use 100 km/h to find if oversteer or understeer
u = 100 / 3.6; % Convert km/h to m/s

if delta <= 0.06
        Cf = 20000;
        Cr = 20000;
    elseif delta <= 0.2
        Cf = 100;
        Cr = 100;
    else
        Cf = 0;
        Cr = 0;
end

% System of ODEs
    A = [- (Cf + Cr) / (m * u), - (a * Cf - b * Cr) / (m * u) - u;
     - (a * Cf - b * Cr) / (Iz * u), - (a^2 * Cf + b^2 * Cr) / (Iz * u)];
    % Define A as a 2x2 matrix, with elements indicated in project outline

    B = [Cf / m; a * Cf / Iz]; % Define B as a 2x1 matrix

% Pre-allocate storage
x_rk4 = zeros(2, length(t)); % Define vector
x_rk4(:,1) = x0; % set first point to ICs
psi = zeros(1, length(t)); % yaw rate
X = zeros(1, length(t)); % X position
Y = zeros(1, length(t)); % Y position

% RK4 solver (same as before)
for i = 1:length(t)-1
    k1 = A * x_rk4(:,i) + B * delta;
    k2 = A * (x_rk4(:,i) + 0.5 * dt * k1) + B * delta;
    k3 = A * (x_rk4(:,i) + 0.5 * dt * k2) + B * delta;
    k4 = A * (x_rk4(:,i) + dt * k3) + B * delta;
    x_rk4(:,i+1) = x_rk4(:,i) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    
    % Integrate yaw rate to get heading angle
    psi(i+1) = psi(i) + dt * x_rk4(2,i);
    
    % Integrate velocity equations to get X and Y positions
    X(i+1) = X(i) + dt * (u * cos(psi(i)) - (x_rk4(1,i) ...
        + a * x_rk4(2,i)) * sin(psi(i)));
    Y(i+1) = Y(i) + dt * ((x_rk4(1,i) + a * x_rk4(2,i)) * cos(psi(i)) ...
        + u * sin(psi(i)));
    % 2 lines above refined and corrected with the use of LLM
end

% Plot Vehicle Trajectory
figure; % open new figure
grid on; % enable grid for clarity
plot(X, Y, 'b', 'LineWidth', 2); % Plot trajectory in blue
xlabel('X Position (m)'); % X-axis label
ylabel('Y Position (m)'); % y-axis label
axis equal; % Keep x and y axes equal when zooming in and out