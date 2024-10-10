% fprintf('Homework 03 Problem 1\n\n\n')
% Author: Anant Girdhar
% Date: 2024-10-08

set(groot, 'defaultLineLineWidth', 2.0);

% Part (a)
% Set the initial conditions
omega_0 = [0.004, 0.004, 0]';  % rad/s
q_0 = [0, 0, 0, 1]';
tspan = [0, 16000];  % seconds
y0 = [omega_0 ; q_0];
[tsim, ysim] = ode45(@eulerseqns1_mod, tspan, y0);

figure('Position', [0, 0, 1400, 600])
plot(tsim, ysim(:, 4))
hold on;
plot(tsim, ysim(:, 5))
plot(tsim, ysim(:, 6))
plot(tsim, ysim(:, 7))
xlim([0, 16000]);
ylim([-1, 1]);
title('Quaternion $\underline{q}_{BI}$ using ode45 - Anant Girdhar', 'Interpreter', 'latex')
xlabel('time (s)')
ylabel('Quaternion components')
legend('$q_1$', '$q_2$', '$q_3$', '$q_4$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
hold off;
saveas(gcf, 'p1_quaternion_ode45.jpg')

figure('Position', [0, 0, 1400, 600])
plot(tsim, ysim(:, 1))
hold on;
plot(tsim, ysim(:, 2))
plot(tsim, ysim(:, 3))
hold off;
xlim([0, 16000]);
ylim([-0.006, 0.006]);
title('Angular velocity $\underline{\omega}_{B}^{BI}$ using ode45 - Anant Girdhar', 'Interpreter', 'latex')
xlabel('time (s)')
ylabel('Angular velocity components')
legend('$\omega_1$', '$\omega_2$', '$\omega_3$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
saveas(gcf, 'p1_angular_velocity_ode45.jpg')

% Recreate the J matrix for the spacecraft
m=10;     % mass in kg
h=0.34;   % height in m  (x)
w=0.2;   % width in m (y)
d=0.1;  % depth in m (z)
Jx=m/12*(w^2+d^2);
Jy=m/12*(h^2+d^2);
Jz=m/12*(h^2+w^2);
J=[Jx,0,0;0,Jy,0;0,0,Jz];
h = J * ysim(:, 1:3)';
h_mag = vecnorm(h)';
figure('Position', [0, 0, 1400, 600])
plot(tsim, h_mag)
title('Magnitude of Angular Momentum using ode45 - Anant Girdhar')
xlabel('time (s)')
ylabel('$|\underline{h}| = |J\underline{\omega}|$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
saveas(gcf, 'p1_angular_momentum_mag_ode45.jpg')

% Part (b)
tsim = 0:1:16000;  % seconds
[ysim] = ode4(@eulerseqns1_mod, tsim, y0);

figure('Position', [0, 0, 1400, 600])
plot(tsim, ysim(:, 4))
hold on;
plot(tsim, ysim(:, 5))
plot(tsim, ysim(:, 6))
plot(tsim, ysim(:, 7))
xlim([0, 16000]);
ylim([-1, 1]);
title('Quaternion $\underline{q}_{BI}$ using ode4 - Anant Girdhar', 'Interpreter', 'latex')
xlabel('time (s)')
ylabel('Quaternion components')
legend('$q_1$', '$q_2$', '$q_3$', '$q_4$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
hold off;
saveas(gcf, 'p1_quaternion_ode4.jpg')

figure('Position', [0, 0, 1400, 600])
plot(tsim, ysim(:, 1))
hold on;
plot(tsim, ysim(:, 2))
plot(tsim, ysim(:, 3))
hold off;
xlim([0, 16000]);
ylim([-0.006, 0.006]);
title('Angular velocity $\underline{\omega}_{B}^{BI}$ using ode4 - Anant Girdhar', 'Interpreter', 'latex')
xlabel('time (s)')
ylabel('Angular velocity components')
legend('$\omega_1$', '$\omega_2$', '$\omega_3$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
saveas(gcf, 'p1_angular_velocity_ode4.jpg')

% Recreate the J matrix for the spacecraft
m=10;     % mass in kg
h=0.34;   % height in m  (x)
w=0.2;   % width in m (y)
d=0.1;  % depth in m (z)
Jx=m/12*(w^2+d^2);
Jy=m/12*(h^2+d^2);
Jz=m/12*(h^2+w^2);
J=[Jx,0,0;0,Jy,0;0,0,Jz];
h = J * ysim(:, 1:3)';
h_mag = vecnorm(h)';
figure('Position', [0, 0, 1400, 600])
plot(tsim, h_mag)
title('Magnitude of Angular Momentum using ode4 - Anant Girdhar')
xlabel('time (s)')
ylabel('$|\underline{h}| = |J\underline{\omega}|$', 'Interpreter', 'latex')
set(gca, 'fontsize', 18)
saveas(gcf, 'p1_angular_momentum_mag_ode4.jpg')

close all
