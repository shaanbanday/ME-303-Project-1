% ME 303 - Zhao Pan
% Programmers: Shaan B, Zubair H, Mirza M, Dharmik R, Milind K
% Date: 11th March, 2025

clc; % clear command window

% Table 1 Parameters
m = 1450;   % Vehicle mass (kg)
a = 1.1; % Distance from centre of mass to the front axle (m)
b = 1.37;  % Distance from centre of mass to the rear axle (m)
Cf = 25000; % Front tire cornering stiffness (N/rad)
Cr = 21000; % Rear tire cornering stiffness (N/rad)
Iz = 2420;  % Yaw inertia (kg·m^2)

% We skip 0 km/h to avoid dividing by zero in A
speeds_kmh = 0.1:0.01:300; % test speeds from 0.1 to 300 km/h
highest_stable_speed = 0; % store the highest stable speed (km/h)

for speed_kmh = speeds_kmh
    u = speed_kmh / 3.6; % convert km/h to m/s

    % System matrix A (2x2)
    A = [ -(Cf + Cr)/(m*u),      -(a*Cf - b*Cr)/(m*u) - u;
          -(a*Cf - b*Cr)/(Iz*u), -(a^2*Cf + b^2*Cr)/(Iz*u) ];
    % We only check the determinant here
    % If det(A) <= 0, at least one eigenvalue is non-negative => unstable.

    if det(A) > 0
        % still stable, update highest stable speed
        highest_stable_speed = speed_kmh;
    else
        % As soon as det(A) <= 0, the system is unstable, stop searching
        break;
    end
end

fprintf('Highest stable speed is %.2f km/h.\n', ...
    highest_stable_speed);
