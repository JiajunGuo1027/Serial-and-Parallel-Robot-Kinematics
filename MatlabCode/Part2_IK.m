% Parameter initialization
SA = 170;
L = 130;
r_platform = 130;
r_base = 290;

% Base joint coordinates
PB = [r_base*cos([pi/2, pi+pi/6, 2*pi-pi/6]); 
      r_base*sin([pi/2, pi+pi/6, 2*pi-pi/6])];

% Platform position
X_c = 0; Y_c = 100; alpha = deg2rad(30);
R_BC = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
PP_local = [r_platform*cos([pi/2, pi+pi/6, 2*pi-pi/6]);
            r_platform*sin([pi/2, pi+pi/6, 2*pi-pi/6])];
PP = R_BC * PP_local + [X_c; Y_c];

% Inverse kinematics solution
theta = zeros(1, 3);
for i = 1:3
    x_PP = PP(1, i);
    y_PP = PP(2, i);
    e1 = -2 * y_PP * SA;
    e2 = -2 * x_PP * SA;
    e3 = x_PP^2 + y_PP^2 + SA^2 - L^2;
    t = roots([e3-e2, 2*e1, e3+e2]);
    theta(i) = 2 * atan(t(1));
end

% Output active joint angles
fprintf('Active joint angles:\n');
for i = 1:3
    fprintf('θ_%d = %.4f radians\n', i, theta(i));
    fprintf('θ_%d = %.4f degrees\n', i, rad2deg(theta(i))); % Optional, convert to degrees
end

% Draw the robot model
figure;
hold on; grid on; axis equal;
title('Planar Parallel Robot Kinematics Model');
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
