fprintf('Homework 04 Problem 5\n\n')
% Author: Anant Girdhar
% Date: 2024-10-27

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Load the results from problem 4
load('P4_results.mat')

fprintf('Part (b)\n\n')
% Compute the small error angle for TRIAD 1
dA_err_1 = true_A_BR_TRIAD_1 * estimated_A_BR_TRIAD_1';
D_hat_1 = eye(3) - dA_err_1;
D_1 = 0.5 * (D_hat_1 - D_hat_1');
dtheta_err_1 = [D_1(3, 2), D_1(1, 3), D_1(2, 1)]';
dtheta_err3a_1 = vecnorm(dtheta_err_1);
fprintf('dtheta_err_1 (in degrees) = \n')
disp(rad2deg(dtheta_err_1))
fprintf('dtheta_err3a_1 = %f degrees\n\n\n', rad2deg(dtheta_err3a_1))

fprintf('Part (d)\n\n')
% Estimate the small error angle for TRIAD 2
dA_err_2 = true_A_BR_TRIAD_2 * estimated_A_BR_TRIAD_2';
D_hat_2 = eye(3) - dA_err_2;
D_2 = 0.5 * (D_hat_2 - D_hat_2');
dtheta_err_2 = [D_2(3, 2), D_2(1, 3), D_2(2, 1)]';
dtheta_err3a_2 = vecnorm(dtheta_err_2);
fprintf('dtheta_err_2 (in degrees) = \n')
disp(rad2deg(dtheta_err_2))
fprintf('dtheta_err3a_2 = %f degrees\n', rad2deg(dtheta_err3a_2))
