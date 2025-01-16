% Define DH parameters
a = [0, 10, 10, 0]; % Link lengths
alpha = [pi/2, 0, 0, 0]; % Twist angles
d = [5, 0, 0, 0]; % Offsets
theta = sym('theta', [1, 4], 'real'); % Joint variables

% Compute DH transformation matrix
DH = @(a, alpha, d, theta) [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];

% Construct Jacobian matrix for the robotic arm
T = eye(4); % Initial transformation matrix
z = [0; 0; 1]; % z-axis direction of the base frame
p = [0; 0; 0]; % Initial position of the end-effector
J = sym(zeros(6, 4)); % Initialize Jacobian matrix

for i = 1:length(a)
    T = T * DH(a(i), alpha(i), d(i), theta(i));
    z_curr = T(1:3, 3); % Current z-axis direction
    p_curr = T(1:3, 4); % Current position of the end-effector
    Jv = cross(z_curr, p_curr - p); % Linear velocity component
    Jw = z_curr; % Angular velocity component
    J(:, i) = [Jv; Jw]; % Add to the Jacobian matrix
end

disp('Jacobian Matrix:');
disp(J);

% Define joint variable ranges
theta1_range = linspace(-pi, pi, 100); % Assume only the first joint is analyzed
detJ_vals = zeros(size(theta1_range)); % Initialize storage for determinant values

% Compute the determinant of the Jacobian matrix at each point
for i = 1:length(theta1_range)
    theta_vals = [theta1_range(i), 0, 0, 0]; % Fix other joints to 0
    J_numeric = double(subs(J, theta, theta_vals)); % Substitute symbolic variables
    detJ_vals(i) = det(J_numeric(1:3, 1:3)); % Compute determinant of the top 3x3 matrix
end

% Plot singularity detection
figure;
plot(theta1_range, detJ_vals);
xlabel('Joint 1 Angle (rad)');
ylabel('Determinant of Jacobian Matrix');
title('Singularity Detection');
grid on;

% Define target points
waypoints = [0, 0, 5; 10, 0, 10; 15, 5, 10];

% Simple path planning (example)
path = waypoints; % Example path, avoiding complex planning logic

% Visualize the path
figure;
plot3(path(:, 1), path(:, 2), path(:, 3), 'r-o', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Path Planning Visualization');
grid on;
