fprintf('Homework 05 Problem 4\n\n')
% Author: Anant Girdhar
% Date: 2024-10-28

% Load relevant utilities
addpath(genpath('../utils/'))
addpath(genpath('../kalman/'))

set(groot, 'defaultLineLineWidth', 2.0);

% % Load the results from problem 3
% load('P3_results.mat')
% true_A_BR_TRIAD_1 = estimated_A_BR_TRIAD_moon;
% true_A_BR_TRIAD_2 = estimated_A_BR_TRIAD_sun;
% clearvars estimated_A_BR_TRIAD_moon estimated_A_BR_TRIAD_sun;

fprintf('Part (a)\n')
% Initialize Kalman filter state
qp = [0, 0, 0, 1]'
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
