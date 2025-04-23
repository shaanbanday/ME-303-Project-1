% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 9th March, 2025

clc; % clear command window

% 100 equally spaced points on the domain x ∈ [0, 5]
t = linspace(0, 5, 50);

% Define the functions y(t) and psi(t) as the "ground truth"
y_dot = -13.096*exp(-1.9745*t) + 24.468*exp(-0.9839*t) - 11.3720;
psi_dot = -0.2496*exp(-1.9745*t) - 0.6962*exp(-0.9839*t) + 0.9457;

% Plotting lateral velocity results
figure; % open a new figure window
grid on; % Turn on the grid for clarity
subplot(2,1,1); % put this plot at the top
plot(t, y_dot, 'LineWidth', 1.5); % Plot y dot
hold on;

% Add x and y axis label
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'Latex');

% Plotting yaw rate results
subplot(2,1,2); % put this plot at the bottom
plot(t, psi_dot, 'LineWidth', 1.5); % Plot psi dot

% Add x and y axis label
xlabel('Time (s)', 'Interpreter', 'Latex');
ylabel('$\dot{\psi}$ (rad/s)', 'Interpreter', 'Latex');
% Similar Latex interpretation as Part 1

hold off; % Release the plot hold