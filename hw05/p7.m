fprintf('Homework 05 Problem 7\n\n')
% Author: Anant Girdhar
% Date: 2024-10-28

% Load relevant utilities
addpath(genpath('../utils/'))
addpath(genpath('../kalman/'))

set(groot, 'defaultLineLineWidth', 2.0);
set(groot, 'defaultAxesFontSize', 18)

% Load the given data
% Loaded variables: b1m, b2m, wm, r1, r2
load('A5P7.mat');
% Load the results from Homework 04 Problem 4
load('A4P4_results.mat', 'q_BR_davenport_2');
q0 = q_BR_davenport_2;
clearvars q_BR_davenport_2;

% Figure out the sizes of the data
Nmeas = size(b1m, 2);
Nsensors = 2;
Nstate = 6;

% Create variables to store the temporal evolution
qp = zeros(4, Nmeas+1);
Pp = zeros(Nstate, Nstate, Nmeas+1);
biasp = zeros(3, Nmeas+1);

% Initialize the Kalman filter state
dt = 1;
tspan = 0:dt*Nmeas;
Y = eye(Nstate);
sigma_v = deg2rad(0.1 / 3600);  % rad/sec
sigma_u = deg2rad(0.1) / 3600;  % rad/sec
Q = Q_drifting_gyro(sigma_v, sigma_u, dt);

sigma_1 = deg2rad(10) / sqrt(3);  % rad
sigma_2 = deg2rad(1) / sqrt(3);  % rad
sigma_sensors = [sigma_1; sigma_2];
R = R_isotropic_sensors(sigma_sensors);

qp(:, 1) = q0;
biasp(:, 1) = zeros(3, 1);
% Construct the initial estimate of the state covariance matrix
% It is a block diagonal matrix
Pp(1:3, 1:3, 1) = ((deg2rad(5) / sqrt(3)) ^ 2) * eye(3);
Pp(4:6, 4:6, 1) = (sigma_u ^ 2) * eye(3);

r_vectors = [r1; r2];
b_vectors = [b1m; b2m];

% Run the Kalman filter
for i = 1:Nmeas
  disp(i)
  % Propagate the solution
  what = wm(:, i) - biasp(:, i);
  Phi = Phi_drifting_gyro(what, dt);
  [qm, Pm] = propagate_attitude(qp(:, i), what, Pp(:, :, i), Phi, Y, Q, dt);
  % Peform the measurement update
  Aqm = quat2DCM(qm);
  hm = h(Aqm, r_vectors);
  Hm = H_drifting_gyro(Aqm, r_vectors);
  K = kalman_gain(Hm, Pm, R);
  dtheta_m = zeros(3, 1);
  dbias_m = zeros(3, 1);
  dx_m = [dtheta_m; dbias_m];
  y = b_vectors(:, i);
  [qp_ip, Pp_ip, dx_ip] = incorporate_measurement(qm, dx_m, y, hm, Hm, Pm, R);
  qp(:, i + 1) = qp_ip;
  Pp(:, :, i + 1) = Pp_ip;
  biasp(:, i + 1) = biasp(:, i) + dx_ip(4:6, 1);
end

save('A5P7_results.mat', ...
  'qm', 'Pm', 'qp', 'Pp', 'biasp' ...
  )

% load('A5P7_results.mat')

% Create a plot of the estimated quaternion elements
figure('Position', [0 0 1400 600])
subplot(411)
plot(tspan(:), qp(1, :))
ylabel('q_{(1)}')
title('Elements of the quaternion, $\hat{\underline{q}}_k^+$', 'Interpreter', 'latex')
subplot(412)
plot(tspan(:), qp(2, :))
ylabel('q_{(2)}')
subplot(413)
plot(tspan(:), qp(3, :))
ylabel('q_{(3)}')
subplot(414)
plot(tspan(:), qp(4, :))
ylabel('q_{(4)}')
xlabel('time (s)')
saveas(gcf, 'p7_quaternion.jpg')

% Create a plot of the estimated individual component attitudes in degrees
figure('Position', [0 0 1400 600])
subplot(311)
plot(tspan, rad2deg(biasp(1, :)) * 3600)  % deg/hr
ylabel('Component 1', 'Interpreter', 'latex')
title('Components of bias $\underline{\hat{\beta}}_k^+$', 'Interpreter', 'latex')
subplot(312)
plot(tspan, rad2deg(biasp(2, :)) * 3600)  % deg/hr
ylabel('Component 2', 'Interpreter', 'latex')
subplot(313)
plot(tspan, rad2deg(biasp(3, :)) * 3600)  % deg/hr
ylabel('Component 3', 'Interpreter', 'latex')
xlabel('time (s)')
saveas(gcf , 'p7_biases.jpg')
