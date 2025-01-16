% Set search range
X_range = -300:10:300; % X direction range
Y_range = -300:10:300; % Y direction range
alpha_range = deg2rad(0:5:60); % Rotation angle range (in radians)

% Initialize workspace storage
workspace_3D = [];

% Iterate over all (X_c, Y_c, alpha) combinations
for a = alpha_range
    for i = 1:length(X_range)
        for j = 1:length(Y_range)
            X_c = X_range(i);
            Y_c = Y_range(j);
            
            % Calculate platform joint positions
            R_BC = [cos(a), -sin(a); sin(a), cos(a)];
            PP_local = [r_platform*cos([pi/2, pi+pi/6, 2*pi-pi/6]);
                        r_platform*sin([pi/2, pi+pi/6, 2*pi-pi/6])];
            PP = R_BC * PP_local + [X_c; Y_c];
            
            % Check feasibility of inverse kinematics solutions
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
            
            % If the current point is feasible, store it in the workspace
            if is_feasible
                workspace_3D = [workspace_3D; X_c, Y_c, rad2deg(a)];
            end
        end
    end
end

% Plot the 3D workspace
figure;
scatter3(workspace_3D(:, 1), workspace_3D(:, 2), workspace_3D(:, 3), 5, 'b', 'filled');
xlabel('X_c (mm)');
ylabel('Y_c (mm)');
zlabel('\alpha (degrees)');
title('3D Workspace of Parallel Robot');
grid on;
