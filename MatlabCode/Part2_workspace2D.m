% Define multiple angles
alpha_values = deg2rad([0, 30, 45, 60]); % Angles in radians

% Iterate over angles to plot workspace
figure;
for idx = 1:length(alpha_values)
    alpha = alpha_values(idx);
    
    % Initialize 2D workspace storage
    workspace_2D = zeros(length(X_range), length(Y_range));
    
    % Iterate over all (X_c, Y_c)
    for i = 1:length(X_range)
        for j = 1:length(Y_range)
            X_c = X_range(i);
            Y_c = Y_range(j);
            
            % Calculate platform joint positions
            R_BC = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
            PP_local = [r_platform*cos([pi/2, pi+pi/6, 2*pi-pi/6]);
                        r_platform*sin([pi/2, pi+pi/6, 2*pi-pi/6])];
            PP = R_BC * PP_local + [X_c; Y_c];
            
            % Check the feasibility of inverse kinematics solutions
            is_feasible = true;
            for k = 1:3
                x_PP = PP(1, k);
                y_PP = PP(2, k);
                e1 = -2 * y_PP * SA;
                e2 = -2 * x_PP * SA;
                e3 = x_PP^2 + y_PP^2 + SA^2 - L^2;
                discriminant = e1^2 - (e3 - e2) * (e3 + e2);
                if discriminant < 0
                    is_feasible = false;
                    break;
                end
            end
            
            % If the current point is feasible, mark it as part of the workspace
            if is_feasible
                workspace_2D(i, j) = 1;
            end
        end
    end
    
    % Plot the 2D workspace
    subplot(2, 2, idx);
    imagesc(X_range, Y_range, workspace_2D');
    set(gca, 'YDir', 'normal'); % Adjust Y-axis direction
    colormap([1 1 1; 0 0 1]);   % White for infeasible, blue for feasible
    title(sprintf('Workspace for \\alpha = %.1f°', rad2deg(alpha)));
    xlabel('X_c (mm)');
    ylabel('Y_c (mm)');
end
