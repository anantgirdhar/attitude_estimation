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

  wx = cross_matrix(what);
  speed = vecnorm(what);
  s = sin(speed * dt);
  c = cos(speed * dt);

  p11 = eye(3) - wx * s / speed + (wx ^ 2) * (1 - c) / (speed ^ 2);
  p12 = wx * (1 - c) / (speed ^ 2) - eye(3) * dt - (wx ^ 2) * (speed * dt - s) / (speed ^ 3);
  p21 = zeros(3, 3);
  p22 = eye(3);
  
  Phi = [p11, p12; p21, p22];

  % [qm, Pm] = propagate_attitude(qp(:, i), what, Pp(:, :, i), Phi, Y, Q, dt);

  psi = sin(0.5 * speed * dt) * what / speed;
  c = cos(0.5 * speed * dt);
  A = c * eye(3) - cross_matrix(psi);

  % Assemble T
  T = [A , psi];
  T = [T ; -psi', c];
  qm = T * qp;
  Pm = Phi * Pp(:, :, i) * Phi' + Y * Q * Y';

  % Peform the measurement update
  % Aqm = quat2DCM(qm);
  q = qm;
  q1sq = q(1) ^ 2;
  q2sq = q(2) ^ 2;
  q3sq = q(3) ^ 2;
  q4sq = q(4) ^ 2;
  q1q2 = q(1) * q(2);
  q1q3 = q(1) * q(3);
  q1q4 = q(1) * q(4);
  q2q3 = q(2) * q(3);
  q2q4 = q(2) * q(4);
  q3q4 = q(3) * q(4);
  % Then assemble the matrix entries
  A11 =  q1sq - q2sq - q3sq + q4sq;
  A22 = -q1sq + q2sq - q3sq + q4sq;
  A33 = -q1sq - q2sq + q3sq + q4sq;
  A12 = 2 * (q1q2 + q3q4);
  A21 = 2 * (q1q2 - q3q4);
  A13 = 2 * (q1q3 - q2q4);
  A31 = 2 * (q1q3 + q2q4);
  A23 = 2 * (q2q3 + q1q4);
  A32 = 2 * (q2q3 - q1q4);
  Aqm = [ ...
    A11 A12 A13; ...
    A21 A22 A23; ...
    A31 A32 A33 ...
    ];

  % hm = h(Aqm, r_vectors);
  m = length(r_vectors) / 3;
  hm = r_vectors;
  for i = 1:m
    hm(3*i-2:3*i, 1) = A * hm(3*i-2:3*i, 1);
  end

  % Hm = H_drifting_gyro(Aqm, r_vectors);
  Hm = zeros(3*m, 6);
  for i = 1:m
    Hm(3*i-2:3*i, 1:3) = cross_matrix(A * r_vectors(3*i-2:3*i, 1));
  end

  % K = kalman_gain(Hm, Pm, R);
  K = Pm * Hm' * inv(Hm * Pm * Hm' + R);

  dtheta_m = zeros(3, 1);
  dbias_m = zeros(3, 1);
  dx_m = [dtheta_m; dbias_m];
  y = b_vectors(:, i);

  % [qp_ip, Pp_ip, dx_ip] = incorporate_measurement(qm, dx_m, y, hm, Hm, Pm, R);
  n = length(dx_m);
  I = eye(n);

  K = kalman_gain(Hm, Pm, R);
  dx_ip = dx_m + K * (y - hm);
  Pp_ip = (I - K * Hm) * Pm;

  dtheta_p = dx_ip(1:3, 1);
  qstar = qm + 0.5 * Xi(qm) * dtheta_p;
  qp_ip = qstar / vecnorm(qstar);

  qp(:, i + 1) = qp_ip;
  Pp(:, :, i + 1) = Pp_ip;
  biasp(:, i + 1) = biasp(:, i) + dx_ip(4:6, 1);
end

% save('A5P7_results.mat', ...
%   'qm', 'Pm', 'qp', 'Pp', 'biasp' ...
%   )
