clc;
clear;
close all;

d1 = 0.1; l1 = 0.2; l2 = 0.2; l3 = 0.1; 

targets = [
    0.2                , 0.0                , 0.2, -pi/2    ;           %1
    0.1                 , 0.0               , 0.4, -pi/4    ;           %2
    0.0                 , 0.0               , 0.6, pi/2     ;           %3
    -0.2                , 0.0               , 0.4, 0    ;           %4
    -0.3               , 0.0               , 0.2, -pi/2    ];

obstacle_center = [-0.2, 0.0, 0.6];
obstacle_radius = 0.15;          

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


plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 

%OBSTACLE
[obstacle_x, obstacle_y, obstacle_z] = sphere(20); % 
obstacle_x = obstacle_x * obstacle_radius + obstacle_center(1);
obstacle_y = obstacle_y * obstacle_radius + obstacle_center(2);
obstacle_z = obstacle_z * obstacle_radius + obstacle_center(3);
surf(obstacle_x, obstacle_y, obstacle_z, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 


draw_arm = @(P1, P2, P3, P4, P5) plot3(... 
    [0, P1(1), P2(1), P3(1), P4(1), P5(1)], ... 
    [0, P1(2), P2(2), P3(2), P4(2), P5(2)], ...
    [0, P1(3), P2(3), P3(3), P4(3), P5(3)], ...
    '-o', 'LineWidth', 2, 'MarkerSize', 6);
trajectory = []; 


for i = 1:size(targets, 1)-1
    start_point = targets(i, :);
    end_point = targets(i+1, :);

    % 1---------2
    if i == 1
        num_steps = 50; % STEP
        interpolated_points = [linspace(start_point(1), end_point(1), num_steps)', ...
                               linspace(start_point(2), end_point(2), num_steps)', ...
                               linspace(start_point(3), end_point(3), num_steps)', ...
                               linspace(start_point(4), end_point(4), num_steps)'];
        for j = 1:num_steps
            x = interpolated_points(j, 1);
            y = interpolated_points(j, 2);
            z = interpolated_points(j, 3);
            theta_fixed = interpolated_points(j, 4);

            offset_x = sin(j / num_steps * pi) * 0.1;  % SIN
            x = x + offset_x;

            theta1 = atan2(y, x);
            xv = x / cos(theta1); yv = z - d1;
            [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(xv, yv, l1, l2, l3, theta_fixed);
            xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
            yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
            zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;
            P5 = [xx,yy,zz];
            P4 = [(l1*cos(theta2)+l2*cos(theta2+theta3))*cos(theta1), ...
                (l1*cos(theta2)+l2*cos(theta2+theta3))*sin(theta1), ...
                l1*sin(theta2)+l2*sin(theta2+theta3)+d1];
            P3 = [(l1*cos(theta2))*cos(theta1), (l1*cos(theta2))*sin(theta1), l1*sin(theta2)+d1];
            P2 = [0,0,d1];
            P1 = [0,0,0];

            trajectory = [trajectory; P5];

            cla;
            draw_arm(P1, P2, P3, P4, P5);
            surf(obstacle_x, obstacle_y, obstacle_z, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 
            plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
            pause(0.02); 
        end
    % 2--------3
    elseif i == 2 || i == 4
        num_steps = 50; % STEP
        interpolated_points = [linspace(start_point(1), end_point(1), num_steps)', ...
                               linspace(start_point(2), end_point(2), num_steps)', ...
                               linspace(start_point(3), end_point(3), num_steps)', ...
                               linspace(start_point(4), end_point(4), num_steps)'];
        for j = 1:num_steps
            % AIMING
            x = interpolated_points(j, 1);
            y = interpolated_points(j, 2);
            z = interpolated_points(j, 3);
            theta_fixed = interpolated_points(j, 4);

            % IK
            theta1 = atan2(y, x);
            xv = x / cos(theta1); yv = z - d1;
            [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(xv, yv, l1, l2, l3, theta_fixed);
            xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
            yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
            zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;
            P5 = [xx,yy,zz];
            P4 = [(l1*cos(theta2)+l2*cos(theta2+theta3))*cos(theta1), ...
                (l1*cos(theta2)+l2*cos(theta2+theta3))*sin(theta1), ...
                l1*sin(theta2)+l2*sin(theta2+theta3)+d1];
            P3 = [(l1*cos(theta2))*cos(theta1), (l1*cos(theta2))*sin(theta1), l1*sin(theta2)+d1];
            P2 = [0,0,d1];
            P1 = [0,0,0];

            % TRAJECTORY KEEPER
            trajectory = [trajectory; P5];

            % DRAWING
            cla;
            draw_arm(P1, P2, P3, P4, P5);
            surf(obstacle_x, obstacle_y, obstacle_z, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 
            plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
            pause(0.02);
        end
    % 3-----4
    elseif i == 3
        num_steps = 100; % STEP
        avoid = 0;
        collapse_value = collapse_check(obstacle_center, obstacle_radius, start_point, end_point);
        for j = 1:num_steps
            if collapse_value == 1 %collapse happen
                avoid = avoid + 1;
                interpolated_points = [linspace(start_point(3), end_point(3), num_steps)', ...
                                       linspace(start_point(4), end_point(4), num_steps)'];
    
                % AIMING POINT
                x = start_point(1);
                y = start_point(2);
                z = interpolated_points(avoid, 1);
                theta_fixed = interpolated_points(avoid, 2);
    
                % IK
                theta1 = atan2(y, x);
                xv = x / cos(theta1); yv = z - d1;
                [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(xv, yv, l1, l2, l3, theta_fixed);
                xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
                yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
                zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;
                P5 = [xx,yy,zz];
                P4 = [(l1*cos(theta2)+l2*cos(theta2+theta3))*cos(theta1), ...
                    (l1*cos(theta2)+l2*cos(theta2+theta3))*sin(theta1), ...
                    l1*sin(theta2)+l2*sin(theta2+theta3)+d1];
                P3 = [(l1*cos(theta2))*cos(theta1), (l1*cos(theta2))*sin(theta1), l1*sin(theta2)+d1];
                P2 = [0,0,d1];
                P1 = [0,0,0];
    
                % TRAJECTORY KEEPER
                trajectory = [trajectory; P5];
    
                % DRAWING
                cla;
                draw_arm(P1, P2, P3, P4, P5);
                surf(obstacle_x, obstacle_y, obstacle_z, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
                plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 
                plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
                pause(0.02); 
                new_start_point = [start_point(1),start_point(2),interpolated_points(avoid, 1),interpolated_points(avoid, 2)];
                collapse_value = collapse_check(obstacle_center, obstacle_radius, new_start_point, end_point);
    
            else
            end
        end
        num_count = num_steps - avoid;
        for j = 1:num_count
            interpolated_points = [linspace(new_start_point(1), end_point(1), num_count)', ...
                               linspace(new_start_point(2), end_point(2), num_count)', ...
                               linspace(new_start_point(3), end_point(3), num_count)', ...
                               linspace(new_start_point(4), end_point(4), num_count)'];
            x = interpolated_points(j, 1);
            y = interpolated_points(j, 2);
            z = interpolated_points(j, 3);
            theta_fixed = interpolated_points(j, 4);

            theta1 = atan2(y, x);
            xv = x / cos(theta1); yv = z - d1;
            [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(xv, yv, l1, l2, l3, theta_fixed);
            xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
            yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
            zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;
            P5 = [xx,yy,zz];
            P4 = [(l1*cos(theta2)+l2*cos(theta2+theta3))*cos(theta1), ...
                (l1*cos(theta2)+l2*cos(theta2+theta3))*sin(theta1), ...
                l1*sin(theta2)+l2*sin(theta2+theta3)+d1];
            P3 = [(l1*cos(theta2))*cos(theta1), (l1*cos(theta2))*sin(theta1), l1*sin(theta2)+d1];
            P2 = [0,0,d1];
            P1 = [0,0,0];

            trajectory = [trajectory; P5];

            cla;
            draw_arm(P1, P2, P3, P4, P5);
            surf(obstacle_x, obstacle_y, obstacle_z, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); 
            plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'b-', 'LineWidth', 1.5); 
            plot3(targets(:, 1), targets(:, 2), targets(:, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2); 
            pause(0.02);
            
        end
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

function collapse_value = collapse_check(obstacle_center, obstacle_radius, start_point, end_point)
    A = [start_point(1),start_point(2),start_point(3)];
    B = [end_point(1),end_point(2),end_point(3)];
    P = obstacle_center;
    
    AB = B - A;
    AP = P - A;
    
    cross_product = cross(AB, AP);
    distance = norm(cross_product) / norm(AB);
    
    disp(['DISTANCE: ', num2str(distance)]);
    if distance < obstacle_radius+0.03
        collapse_value = 1;
    else
        collapse_value = 0;
    end
end
