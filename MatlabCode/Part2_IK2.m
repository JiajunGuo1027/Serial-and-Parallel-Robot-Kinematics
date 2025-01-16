% Parameter Initialization
SA = 170; L = 130; r_platform = 130; r_base = 290;
PB = [r_base*cos([pi/2, pi+pi/6, 2*pi-pi/6]); 
      r_base*sin([pi/2, pi+pi/6, 2*pi-pi/6])];

% Define multiple Cartesian input parameters (X_c, Y_c, alpha)
inputs = [
    50, 100, deg2rad(0);     % Platform center (X_c = 50, Y_c = 100, α = 0°)
    50, 150, deg2rad(30);    % Platform center (X_c = 50, Y_c = 150, α = 30°)
    -50, 50, deg2rad(-30);   % Platform center (X_c = -50, Y_c = 50, α = -30°)
    100, 150, deg2rad(45);   % Platform center (X_c = 100, Y_c = 150, α = 45°)
];

% Initialize a storage matrix for all joint angles
theta_all = zeros(size(inputs, 1), 3);

% Simulate different platform positions in a loop
figure;
for k = 1:size(inputs, 1)
    % Get input parameters
    X_c = inputs(k, 1);
    Y_c = inputs(k, 2);
    alpha = inputs(k, 3);
    
    % Calculate platform joint positions
    R_BC = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
    PP_local = [r_platform*cos([pi/2, pi+pi/6, 2*pi-pi/6]);
                r_platform*sin([pi/2, pi+pi/6, 2*pi-pi/6])];
    PP = R_BC * PP_local + [X_c; Y_c];

    % Solve inverse kinematics
    theta = zeros(1, 3);
    for i = 1:3
        x_PP = PP(1, i);
        y_PP = PP(2, i);
        e1 = -2 * y_PP * SA;
        e2 = -2 * x_PP * SA;
        e3 = x_PP^2 + y_PP^2 + SA^2 - L^2;
        
        discriminant = e1^2 - (e3 - e2) * (e3 + e2);
        if discriminant < 0
            warning('Discriminant is negative, resulting in complex roots. Skipping calculation for this leg.');
            theta(i) = NaN;
            continue;
        end
        
        t = roots([e3-e2, 2*e1, e3+e2]);
        theta(i) = 2 * atan(t(1));
    end
    
    theta_all(k, :) = theta; % Store joint angles for this position

    % Plot the robot model
    subplot(2, 2, k);
    hold on; grid on; axis equal;
    title(sprintf('Position %d: X_c=%.1f, Y_c=%.1f, α=%.1f°', k, X_c, Y_c, rad2deg(alpha)));
    xlabel('X (mm)'); ylabel('Y (mm)');
    plot([PB(1, :) PB(1, 1)], [PB(2, :) PB(2, 1)], 'k-o', 'LineWidth', 2);
    plot([PP(1, :) PP(1, 1)], [PP(2, :) PP(2, 1)], 'r-o', 'LineWidth', 2);
    for i = 1:3
        joint_x = [PB(1, i), PB(1, i) + SA*cos(theta(i))];
        joint_y = [PB(2, i), PB(2, i) + SA*sin(theta(i))];
        plot(joint_x, joint_y, 'b-', 'LineWidth', 2);
        plot([joint_x(2), PP(1, i)], [joint_y(2), PP(2, i)], 'g-', 'LineWidth', 2);
    end
    legend('Base', 'Platform', 'Upper segment', 'Lower segment');
    hold off;
end

% Output all joint angles
fprintf('Active joint angles for different input parameters:\n');
for k = 1:size(inputs, 1)
    fprintf('Position %d (X_c=%.1f, Y_c=%.1f, α=%.1f°):\n', k, inputs(k, 1), inputs(k, 2), rad2deg(inputs(k, 3)));
    for i = 1:3
        fprintf('  θ_%d = %.4f radians (%.4f degrees)\n', i, theta_all(k, i), rad2deg(theta_all(k, i)));
    end
end
