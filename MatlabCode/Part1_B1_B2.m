clc;
clear;
close all;

d1 = 0.1; l1 = 0.2; l2 = 0.2; l3 = 0.1;

% targets = [
%             -0.3,-0.1,0.3,pi/4;
%             0.32,0.0,0.2,-pi/2];
%      0.2                , 0.0               , 0.4, pi/4     ;           %1
%      0.2*cos(pi*8/10)   , 0.2*sin(pi*8/10)  , 0.4, pi/6     ;           %3
%      0.2*cos(pi*16/10)  , 0.2*sin(pi*16/10) , 0.4, 0        ;           %5
%      0.2*cos(pi*4/10)   , 0.2*sin(pi*4/10)  , 0.4, -pi/5    ;           %2
%      0.2*cos(pi*12/10)  , 0.2*sin(pi*12/10) , 0.4, 0        ;           %4
%      0.2                , 0.0               , 0.4, 0];

targets = [
     0.32                , 0.0               , 0.2, -pi/2    ;           %1
     0.2                , 0.0               , 0.4, 0       ;           %3
     0.0                , 0.0               , 0.6, pi/2       ;           %5
     -0.2               , 0.0               , 0.4, 0       ;           %4
     -0.32               , 0.0               , 0.2, -pi/2];

% CONFIG
figure;
hold on;
axis equal;
grid on;
xlim([-0.6, 0.6]);
ylim([-0.6, 0.6]);
zlim([0, 0.8]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('trajectory');

plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % 目标点

draw_arm = @(P1, P2, P3, P4, P5) plot3(... 
    [0, P1(1), P2(1), P3(1), P4(1), P5(1)], ... 
    [0, P1(2), P2(2), P3(2), P4(2), P5(2)], ...
    [0, P1(3), P2(3), P3(3), P4(3), P5(3)], ...
    '-o', 'LineWidth', 2, 'MarkerSize', 6);

trajectory = []; 


for i = 1:size(targets, 1)-1
    start_point = targets(i, :);
    end_point = targets(i+1, :);


    num_steps = 50; 
    interpolated_points = [linspace(start_point(1), end_point(1), num_steps)', ...
                           linspace(start_point(2), end_point(2), num_steps)', ...
                           linspace(start_point(3), end_point(3), num_steps)', ...
                           linspace(start_point(4), end_point(4), num_steps)']; 

    for j = 1:size(interpolated_points, 1)
        x = interpolated_points(j, 1);
        y = interpolated_points(j, 2);
        z = interpolated_points(j, 3);
        theta_fixed = interpolated_points(j, 4); 

        theta1 = atan2(y,x);
    % p in RRR manipulator
        xv = x / cos(theta1); yv = z - d1;
    
        % IK
        [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(xv, yv, l1, l2, l3, theta_fixed);
        xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
        yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
        zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;
        P5 = [xx,yy,zz];
        P4 = [(l1*cos(theta2)+l2*cos(theta2+theta3))*cos(theta1), (l1*cos(theta2)+l2*cos(theta2+theta3))*sin(theta1), l1*sin(theta2)+l2*sin(theta2+theta3)+d1];
        P3 = [(l1*cos(theta2))*cos(theta1), (l1*cos(theta2))*sin(theta1), l1*sin(theta2)+d1];
        P2 = [0,0,d1];
        P1 = [0,0,0];

        trajectory = [trajectory; P5];

        cla;
        draw_arm(P1, P2, P3, P4, P5);
        plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 
        plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
        pause(2); 
    end
end

plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 

function [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(x, y, a1, a2, a3, fixed_theta4)
    theta4 = fixed_theta4;

    x_eff = x - a3 * cos(theta4);
    y_eff = y - a3 * sin(theta4);
    r_eff = sqrt(x_eff^2 + y_eff^2);

    if r_eff > (a1 + a2) || r_eff < abs(a1 - a2)
        error('exceeded');
    end

    cos_theta2 = (x_eff^2 +y_eff^2 - a1^2 - a2^2) / (2 * a1 * a2);
    theta3 = acos(cos_theta2);

%     ss1 = ((a1+a2*cos(theta2))*y_eff-a2*sin(theta2)*x_eff)/(x_eff^2+y_eff^2);
%     cc1 = ((a1+a2*cos(theta2))*x_eff+a2*sin(theta2)*y_eff)/(x_eff^2+y_eff^2);
%     theta1 = atan2(ss1,cc1);
    
    phi = atan2(y_eff, x_eff);
    psi = atan2(a2 * sin(theta3), a1 + a2 * cos(theta3));
    theta2 = phi - psi;
    theta4 = theta4 - theta2 - theta3;
end
