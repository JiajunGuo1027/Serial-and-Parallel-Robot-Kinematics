% Definition of robot arm parameters
d1 = 0.1;    % Distance from the first joint to the second joint
L1 = 0.2;    % Length of the link from the second joint to the third joint
L2 = 0.2;    % Length of the link from the third joint to the fourth joint
L3 = 0.1;    % Distance from the fifth joint to the gripper

% Define joint angles (units: radians)
theta1 = deg2rad(60);
theta2 = deg2rad(45);
theta3 = deg2rad(-30);
theta4 = deg2rad(60);
theta5 = deg2rad(30);

% DH transformation matrix function
function T = DH_matrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% Compute the transformation matrices for each joint
T1 = DH_matrix(theta1, d1, 0, pi/2);
T2 = DH_matrix(theta2, 0, L1, 0);
T3 = DH_matrix(theta3, 0, L2, 0);
T4 = DH_matrix(theta4, 0, 0, pi/2);
T5 = DH_matrix(theta5, L3, 0, 0);

% Final transformation matrix
T_final = T1 * T2 * T3 * T4 * T5;
disp('The final transformation matrix:');
disp(T_final);

% Calculate joint positions
T1_to_2 = T1 * T2;
T1_to_3 = T1_to_2 * T3;
T1_to_4 = T1_to_3 * T4;
origin = [0, 0, 0];
joint1 = T1(1:3, 4)';
joint2 = T1_to_2(1:3, 4)';
joint3 = T1_to_3(1:3, 4)';
joint4 = T1_to_4(1:3, 4)';
end_effector = T_final(1:3, 4)';

% Display the position of the end effector
disp('The position of the end effector:');
disp(end_effector);

% Plot the robotic arm
figure;
plot3([origin(1), joint1(1), joint2(1), joint3(1), joint4(1), end_effector(1)], ...
      [origin(2), joint1(2), joint2(2), joint3(2), joint4(2), end_effector(2)], ...
      [origin(3), joint1(3), joint2(3), joint3(3), joint4(3), end_effector(3)], '-o');
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Forward kinematics simulation of Lynxmotion robotic arm');
