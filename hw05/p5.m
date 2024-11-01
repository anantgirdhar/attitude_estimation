fprintf('Homework 05 Problem 5\n\n')
% Author: Anant Girdhar
% Date: 2024-10-28

% Load relevant utilities
addpath(genpath('../utils/'))
addpath(genpath('../kalman/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Load measureents given in problem
r1 = [0.5, 0, -sqrt(3)/2]';
r2 = [0, 0, -1]';
b1 = [0.7338, 0.1985, 0.6498]';
b2 = [0.9999, 0.0138, -0.0032]';
sigma_m = deg2rad(10) / sqrt(3);  % rad
sigma_s = deg2rad(1) / sqrt(3);  % rad

sigmas = [sigma_m; sigma_s];
r_vectors = [r1; r2];
b_vectors = [b1; b2];

fprintf('Part (a)\n')
% Compute quantities for measurement update step
R = R_isotropic_sensors(sigmas)
Aqm = quat2DCM(qm)
hm = h(Aqm, r_vectors)
Hm = H_calibrated_gyro(Aqm, r_vectors)
K = kalman_gain(Hm, Pm, R)

fprintf('Part (b) and (c)\n')
% Perform the measurement update step
dt = 1;
dtheta_m = zeros(3, 1);
y = b_vectors;
[qp, Pp, dtheta_p] = incorporate_measurement(qm, dtheta_m, y, hm, Hm, Pm, R)
