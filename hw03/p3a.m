% fprintf('Homework 03 Problem 3 (a)\n\n\n')
% Author: Anant Girdhar
% Date: 2024-10-09
% Simulate bang-bang controller

set(groot, 'defaultLineLineWidth', 2.0);

% I don't really like this but I don't want to have to rewrite a bunch of code
% right now
global L LHIST MHIST;

% Compute trajectory and B-field
altitude = 400;  % km
tspan = 0:1:16000;  % seconds
latitudes = zeros(length(tspan));
longitudes = zeros(length(tspan));
B_ned = zeros(length(tspan), 3);
B_eci = zeros(length(tspan), 3);
date_stamp_0 = datenum('October 1, 2024 00:00:00');
date_stamp_second = datenum('October 1, 2024 00:00:01') - datenum('October 1, 2024 00:00:00');
for i = 1:length(tspan)
  [latitudes(i), longitudes(i)] = issorb(tspan(i));  % radians
  latitudes(i) = rad2deg(latitudes(i));  % degrees
  longitudes(i) = rad2deg(longitudes(i));  % degrees
  date_stamp = date_stamp_0 + date_stamp_second * tspan(i);
  B_ned(i, :) = igrf(date_stamp, latitudes(i), longitudes(i), altitude, 'geodetic');  % nT
  B_eci(i, :) = ned2eci(B_ned(i, :)', latitudes(i), longitudes(i), tspan(i))';  % nT
end

% % Verify answer
% load A3P3.mat
% assert(all(abs((B_eci(:, 1) - Bxi) ./ Bxi) < 1e-6))
% assert(all(abs((B_eci(:, 2) - Byi) ./ Byi) < 1e-6))
% assert(all(abs((B_eci(:, 3) - Bzi) ./ Bzi) < 1e-6))
% B_eci = [Bxi Byi Bzi];

% Initialize
omega0 = [0, 0.2, 0.2]';  % rad/s
q0 = [0, 0, 0, 1]';
y0 = [omega0 ; q0];
Y = ode4m(@eulerseqns2, tspan, y0);

% Plot attitude
figure('Position', [0, 0, 1400, 600])
hold on
plot(tspan, Y(:, 4))
plot(tspan, Y(:, 5))
plot(tspan, Y(:, 6))
plot(tspan, Y(:, 7))
xlabel('time (s)')
ylabel('$\underline{q}_{BI_i}$', 'Interpreter', 'latex')
title('Components of $\underline{q}_{BI}$ - Bdot controller - Anant', 'Interpreter', 'latex')
legend('q_1', 'q_2', 'q_3', 'q_4')
set(gca, 'fontsize', 18)
saveas(gcf, 'p3a_q.jpg')

% Plot angular velocity
figure('Position', [0, 0, 1400, 600])
hold on
plot(tspan, Y(:, 1))
plot(tspan, Y(:, 2))
plot(tspan, Y(:, 3))
xlabel('time (s)')
ylabel('$\underline{\omega}_{B_i}^{BI}$ (rad/s)', 'Interpreter', 'latex')
title('Components of $\underline{\omega}_B^{BI}$ - Bdot controller - Anant', 'Interpreter', 'latex')
legend('\omega_1', '\omega_2', '\omega_3')
set(gca, 'fontsize', 18)
saveas(gcf, 'p3a_angular_velocity.jpg')

% Plot angular speed
omega_mag = vecnorm(Y(:, 1:3)')';
figure('Position', [0, 0, 1400, 600])
hold on
plot(tspan, omega_mag)
xlabel('time (s)')
ylabel('$\omega_B^{BI}$ (rad/s)', 'Interpreter', 'latex')
title('Angular speed - Bdot controller - Anant')
set(gca, 'fontsize', 18)
saveas(gcf, 'p3a_angular_speed.jpg')

% Plot commanded magnetic dipole moment
figure('Position', [0, 0, 1400, 600])
hold on
plot(tspan, MHIST(1, :))
plot(tspan, MHIST(2, :))
plot(tspan, MHIST(3, :))
xlabel('time (s)')
ylabel('m_{B_i} (A.m^2)')
title('Commanded magnetic dipole moment for each magnetorquer - Bdot controller - Anant')
legend('m_{B_1}', 'm_{B_2}', 'm_{B_3}')
set(gca, 'fontsize', 18)
saveas(gcf, 'p3a_commanded_dipole.jpg')

close all
