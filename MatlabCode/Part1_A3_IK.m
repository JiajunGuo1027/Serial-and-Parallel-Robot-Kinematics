function joint_angles = inverse_kinematics(x, y, z, d1, L1, L2, L3, R_target)
    % Function to compute inverse kinematics for Lynxmotion 5DOF arm.
    % Inputs:
    %   x, y, z - Target end-effector position
    %   d1 - Distance from base to second joint
    %   L1, L2 - Link lengths
    %   L3 - Distance from wrist to end-effector
    %   R_target - Target orientation (3x3 rotation matrix)
    % Outputs:
    %   joint_angles - Array of joint angles [theta1, theta2, theta3, theta4, theta5]

    % Step 1: Compute theta1 (base rotation)
    theta1 = atan2(y, x);

    % Step 2: Compute r and h
    r = sqrt(x^2 + y^2);
    h = z - d1;

    % Step 3: Compute theta2 and theta3 using geometry
    D = (r^2 + h^2 - L1^2 - L2^2) / (2 * L1 * L2);
    if abs(D) > 1
        error('Target position is out of reach.');
    end
    theta3 = atan2(sqrt(1 - D^2), D); % Elbow down solution
    theta2 = atan2(h, r) - atan2(L2*sin(theta3), L1 + L2*cos(theta3));

    % Step 4: Compute wrist position and orientation
    % Forward kinematics to calculate R_wrist (orientation of the wrist base)
    T1 = DH_matrix(theta1, d1, 0, pi/2);
    T2 = DH_matrix(theta2, 0, L1, 0);
    T3 = DH_matrix(theta3, 0, L2, 0);

    % Rotation matrix of the wrist base
    R_wrist = T1(1:3, 1:3) * T2(1:3, 1:3) * T3(1:3, 1:3);

    % Ensure R_wrist and R_target are valid
    if ~isequal(size(R_wrist), [3, 3]) || ~isequal(size(R_target), [3, 3])
        error('R_wrist or R_target is not a 3x3 matrix');
    end

    % Compute remaining wrist orientation
    R_wrist_remaining = R_wrist' * R_target;

    % Extract theta4 and theta5 from R_wrist_remaining
    theta4 = atan2(R_wrist_remaining(2, 3), R_wrist_remaining(1, 3));
    theta5 = atan2(sqrt(R_wrist_remaining(1, 3)^2 + R_wrist_remaining(2, 3)^2), R_wrist_remaining(3, 3));

    % Output joint angles
    joint_angles = [theta1, theta2, theta3, theta4, theta5];
end

% Parameters
d1 = 0.1; % Height of base to first joint
L1 = 0.2; % Length of first link
L2 = 0.2; % Length of second link
L3 = 0.1; % Distance from wrist to end-effector

% Target position and orientation
x = 0.2; y = 0.2; z = 0.3;
R_target = eye(3); % Assuming target orientation as identity matrix

% Compute joint angles
joint_angles = inverse_kinematics(x, y, z, d1, L1, L2, L3, R_target);

% Display the results
disp('Computed joint angles (in radians):');
disp(joint_angles);
disp('Computed joint angles (in degrees):');
disp(rad2deg(joint_angles));

% DH transformation matrix function
function T = DH_matrix(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% Verify inverse kinematics result using forward kinematics
% Input the joint angles computed by IK
theta1 = joint_angles(1);
theta2 = joint_angles(2);
theta3 = joint_angles(3);
theta4 = joint_angles(4);
theta5 = joint_angles(5);

% Compute forward kinematics
T1 = DH_matrix(theta1, d1, 0, pi/2);
T2 = DH_matrix(theta2, 0, L1, 0);
T3 = DH_matrix(theta3, 0, L2, 0);
T4 = DH_matrix(theta4, 0, 0, pi/2);
T5 = DH_matrix(theta5, L3, 0, 0);

T_final = T1 * T2 * T3 * T4 * T5;

% Extract end-effector position and orientation
end_effector_position = T_final(1:3, 4);       % Extract position
end_effector_orientation = T_final(1:3, 1:3); % Extract orientation matrix

% Compute position error
position_error = norm([x; y; z] - end_effector_position);

% Compute orientation error (if R_target is provided)
orientation_error = norm(R_target - end_effector_orientation, 'fro');

% Display results
disp('Verification Results:');
disp('Target Position:');
disp([x, y, z]);
disp('Computed Position:');
disp(end_effector_position');
disp(['Position Error: ', num2str(position_error), ' meters']);

if exist('R_target', 'var')
    disp('Target Orientation (R_target):');
    disp(R_target);
    disp('Computed Orientation (R_computed):');
    disp(end_effector_orientation);
    disp(['Orientation Error: ', num2str(orientation_error)]);
end
