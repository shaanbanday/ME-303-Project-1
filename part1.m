% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 3rd March, 2025

clc; % clear command window

% 100 equally spaced points on the domain x ∈ [0, 5]
x = linspace(0, 5, 50);

% compute solution to ODE using ME 203 analytical method
y_analytical = exp(x);

% compute solution to ODE using power series numerical approx
y_series = zeros(size(x)); % initialize series approx vector
for n = 0:5 % only calculate first five points
    y_series = y_series + (x.^n) / factorial(n); % from summation
    % use .^ since we are doing element-by-element powers
end

figure; % open a new figure window

% Plot the analytical solution as a blue solid line
plot(x, y_analytical, 'b-', 'LineWidth', 1.5);
hold on; % keep the plot on

% Plot the power series approx as red markers connected with lines
plot(x, y_series, 'r.-', 'LineWidth', 1.5, 'MarkerSize', 8);

% Set x-axis ticks to 0, 1, 2, 3, 4, 5
xticks(0:1:5);

% Set y-axis ticks to every 10 until e^5 rounded up to nearest 10
yticks(0:10:round(max(y_analytical),-1));

% Turn on the grid for clarity
grid on;

% Add a legend to distinguish between the two curves
legend('$y=e^{x}$', '$y = \sum\limits_{n=0}^{5} \frac{x^n}{n!}$', ...
    'Interpreter', 'latex', 'Location', 'Best', 'FontSize', 14);
% LaTex inerpretation above was coded by consulting the MATLAB and Latex
% documentation, as well as using an LLM

% Calculate and Print the Error Norms
% Compute the absolute error at each x point
error = y_analytical - y_series;

% Calculate the L-1, L-2, and L-infinity norms of the error vector
L1_norm = norm(error, 1); % sum of absolute errors
L2_norm = norm(error); % square root of sum of squared errors
Linf_norm = abs(max(error)); % take abs value of highest element


% Print the error
fprintf('L1 Norm of Error: %f\n', L1_norm);
fprintf('L2 Norm of Error: %f\n', L2_norm);
fprintf('L-∞ Norm of Error: %f\n', Linf_norm);

hold off; % release the plot hold