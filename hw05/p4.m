fprintf('Homework 05 Problem 4\n\n')
% Author: Anant Girdhar
% Date: 2024-10-28

% Load relevant utilities
addpath(genpath('../utils/'))
addpath(genpath('../kalman/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Load the results from Homework 04 Problem 4
load('A4P4_results.mat', 'q_BR_davenport_2');
q0 = q_BR_davenport_2;
clearvars q_BR_davenport_2;

fprintf('Part (a)\n')
% Initialize Kalman filter state
qp = q0
Pp = ((deg2rad(5) / sqrt(3)) ^ 2) * eye(3)  % squared radians

fprintf('Part (b)\n')
% Initialize Kalman filter dynamics
dt = 1;
omega_meas = [0.6150, 0.0017, -0.5997]' * 1e-3;  % rad/s
Phi = Phi_calibrated_gyro(omega_meas, dt)
sigma_v = deg2rad(0.1 / 3600);  % rad/sec
Q = Q_calibrated_gyro(sigma_v, dt)

fprintf('Part (c)\n')
% Propagate the measurement one time step
Y = eye(3);  % process noise "gain" matrix
[qm, Pm] = propagate_attitude(qp, omega_meas, Pp, Phi, Y, Q, dt)
