% Definition of robot arm parameters
d1 = 0.1;    % Distance from the first joint to the second joint
L1 = 0.2;    % Length of the link from the second joint to the third joint
L2 = 0.2;    % Length of the link from the third joint to the fourth joint
L3 = 0.1;    % Distance from the fifth joint to the gripper

% Define the ranges for joint angles (units: radians)
theta1_range = deg2rad(0:10:360);
theta2_range = deg2rad(-90:10:90);
theta3_range = deg2rad(-90:10:90);
theta4_range = deg2rad(0:10:360);

% Preallocate a matrix to store workspace points
max_points = length(theta1_range) * length(theta2_range) * length(theta3_range) * length(theta4_range);
workspace_points = zeros(max_points, 3);
index = 1;

% DH transformation matrix function
function T = DH_matrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% Iterate through all possible joint angle combinations
for theta1 = theta1_range
    for theta2 = theta2_range
        for theta3 = theta3_range
            for theta4 = theta4_range
                % Compute the transformation matrices
                T1 = DH_matrix(theta1, d1, 0, pi/2);
                T2 = DH_matrix(theta2, 0, L1, 0);
                T3 = DH_matrix(theta3, 0, L2, 0);
                T4 = DH_matrix(theta4, 0, 0, pi/2);
                
                % Compute the transformation to the 5th joint (wrist center)
                T_wrist = T1 * T2 * T3 * T4;
                
                % Extract the position of the wrist center
                wrist_position = T_wrist(1:3, 4)';
                
                % Store the position in the preallocated matrix
                workspace_points(index, :) = wrist_position;
                index = index + 1;
            end
        end
    end
end

% Remove unused rows in the preallocated matrix
workspace_points = workspace_points(1:index-1, :);

% Plot the 3D workspace
figure;
scatter3(workspace_points(:, 1), workspace_points(:, 2), workspace_points(:, 3), '.');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Workspace of Wrist Center');
grid on;
axis equal;

% Plot the 2D projection of the workspace (XY plane)
figure;
scatter(workspace_points(:, 1), workspace_points(:, 2), '.');
xlabel('X (m)'); ylabel('Y (m)');
title('2D Workspace of Wrist Center (XY Plane)');
grid on;
axis equal;
