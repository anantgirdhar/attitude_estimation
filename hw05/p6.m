fprintf('Homework 05 Problem 6\n\n')
% Author: Anant Girdhar
% Date: 2024-10-28

% Load relevant utilities
addpath(genpath('../utils/'))
addpath(genpath('../kalman/'))

set(groot, 'defaultLineLineWidth', 2.0);
set(groot, 'defaultAxesFontSize', 18)

% Load the given data
% Loaded variables: b1m, b2m, wm, r1, r2
load('A5P6.mat');
% Load the results from Homework 04 Problem 4
load('A4P4_results.mat', 'q_BR_davenport_2');
q0 = q_BR_davenport_2;
clearvars q_BR_davenport_2;

% Figure out the sizes of the data
Nmeas = size(b1m, 2);
Nsensors = 2;
Nstate = 3;

% Create variables to store the temporal evolution
qm = zeros(4, Nmeas);
qp = zeros(4, Nmeas+1);
Pm = zeros(Nstate, Nstate, Nmeas);
Pp = zeros(Nstate, Nstate, Nmeas+1);
sigma_P = zeros(3, Nmeas+1);

% Initialize the Kalman filter state
dt = 1;
tspan = 0:dt*Nmeas;
Y = eye(3);
sigma_v = deg2rad(0.1 / 3600);  % rad/sec
Q = Q_calibrated_gyro(sigma_v, dt);

sigma_1 = deg2rad(10) / sqrt(3);  % rad
sigma_2 = deg2rad(1) / sqrt(3);  % rad
sigma_sensors = [sigma_1; sigma_2];
R = R_isotropic_sensors(sigma_sensors);

qp(:, 1) = q0;
Pp(:, :, 1) = ((deg2rad(5) / sqrt(3)) ^ 2) * eye(Nstate);
sigma_P(1, 1) = sqrt(Pp(1, 1, 1));
sigma_P(2, 1) = sqrt(Pp(2, 2, 1));
sigma_P(3, 1) = sqrt(Pp(3, 3, 1));

r_vectors = [r1; r2];
b_vectors = [b1m; b2m];

% Run the Kalman filter
for i = 1:Nmeas
  disp(i)
  % Propagate the solution
  Phi = Phi_calibrated_gyro(wm(:, i), dt);
  [qm_i, Pm_i] = propagate_attitude(qp(:, i), wm(:, i), Pp(:, :, i), Phi, Y, Q, dt);
  qm(:, i) = qm_i;
  Pm(:, :, i) = Pm_i;
  % Peform the measurement update
  Aqm = quat2DCM(qm_i);
  hm = h(Aqm, r_vectors);
  Hm = H_calibrated_gyro(Aqm, r_vectors);
  K = kalman_gain(Hm, Pm(:, :, i), R);
  dtheta_m = zeros(3, 1);
  y = b_vectors(:, i);
  [qp_ip, Pp_ip, dtheta_ip] = incorporate_measurement(qm_i, dtheta_m, y, hm, Hm, Pm(:, :, i), R);
  qp(:, i + 1) = qp_ip;
  Pp(:, :, i + 1) = Pp_ip;
  sigma_P(1, i + 1) = sqrt(Pp(1, 1, i + 1));
  sigma_P(2, i + 1) = sqrt(Pp(2, 2, i + 1));
  sigma_P(3, i + 1) = sqrt(Pp(3, 3, i + 1));
end

save('A5P6_results.mat', ...
  'qm', 'Pm', 'qp', 'Pp', 'sigma_P' ...
  )

% load('A5P6_results.mat')

% Create a plot of the estimated quaternion elements
figure('Position', [0 0 1400 600])
subplot(411)
plot(tspan, qp(1, :))
ylabel('q_{(1)}')
title('Elements of the quaternion, $\hat{\underline{q}}_k^+$', 'Interpreter', 'latex')
subplot(412)
plot(tspan, qp(2, :))
ylabel('q_{(2)}')
subplot(413)
plot(tspan, qp(3, :))
ylabel('q_{(3)}')
subplot(414)
plot(tspan, qp(4, :))
ylabel('q_{(4)}')
xlabel('time (s)')
saveas(gcf, 'p6_quaternion.jpg')

% Create a plot of the estimated individual component attitudes in degrees
figure('Position', [0 0 1400 600])
subplot(311)
plot(tspan, rad2deg(sigma_P(1, :)))
ylabel('$\sigma_x$', 'Interpreter', 'latex')
title('Individual component attitude errors from $P_k^+$', 'Interpreter', 'latex')
subplot(312)
plot(tspan, rad2deg(sigma_P(2, :)))
ylabel('$\sigma_y$', 'Interpreter', 'latex')
subplot(313)
plot(tspan, rad2deg(sigma_P(3, :)))
ylabel('$\sigma_z$', 'Interpreter', 'latex')
xlabel('time (s)')
saveas(gcf , 'p6_attitude_errors.jpg')

% Create a plot of the estimated 3-axis attitude error in degrees
sigma_3a = vecnorm(sigma_P);
figure('Position', [0 0 1400 500])
plot(tspan, rad2deg(sigma_3a))
ylabel('$\sigma_{err3a}$ (degrees)', 'Interpreter', 'latex')
xlabel('time (s)')
title('3-axis Attitude Error')
saveas(gcf, 'p6_3axis_error.jpg')

% What is the steady state value of the 3-axis attitude error in degrees
fprintf('Final value of 3-axis attitude error: %f degrees\n', rad2deg(sigma_3a(end)))
