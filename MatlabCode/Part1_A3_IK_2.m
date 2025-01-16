
xa = 0.3; ya = 0.2; za = 0.35; 
theta1 = atan2(ya,xa);
d1 = 0.1;
l1 = 0.2; l2 = 0.2; l3 = 0.1;

% p in RRR manipulator
x = xa / cos(theta1); y = za - d1;

% IK
[theta2, theta3, theta4] = planar_arm_inverse_no_orientation(x, y, l1, l2, l3);
xx = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*cos(theta1);
yy = (l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta3+theta4))*sin(theta1);
zz = l1*sin(theta2)+l2*sin(theta2+theta3)+l3*sin(theta2+theta3+theta4)+d1;

fprintf('theta1 = %.2f rad\n', theta1);
fprintf('theta2 = %.2f rad\n', theta2);
fprintf('theta3 = %.2f rad\n', theta3);
fprintf('theta4 = %.2f rad\n', theta4);
fprintf('Input: x is %.2f, y is %.3f, and z is %.3f.\n', xa,ya,za);
fprintf('Output:x is %.2f, y is %.3f, and z is %.3f.\n', xx,yy,zz);

function [theta2, theta3, theta4] = planar_arm_inverse_no_orientation(x, y, a1, a2, a3, fixed_theta4)
    fixed_theta4 = pi/4;
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
