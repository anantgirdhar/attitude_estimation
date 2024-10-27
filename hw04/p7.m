fprintf('Homework 04 Problem 7\n\n')
% Author: Anant Girdhar
% Date: 2024-10-20

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Given celestial object directions
r_moon = [0.5, 0, -0.5 * sqrt(3)]';
b_moon = [0.8126, -0.0316, 0.5820]';
r_sun = [0, 0, -1]';
b_sun = [0.9999, -0.0101, -0.0101]';

q_BR_st = [0, 0.707047, 0.000119, 0.707166]';

% Load the results from problem 3
load('P3_results.mat')
true_A_BR_TRIAD = estimated_A_BR_TRIAD_moon;
clearvars estimated_A_BR_TRIAD_moon estimated_A_BR_TRIAD_sun;

fprintf('Part (a)\n')
% Resolve the problem 6 attitude solution with new weights

% Get the star vectors
b_star = [1 0 0]';
r_star = quat2DCM(q_BR_st)' * b_star;

% Set the new weights based on sensor error statistics
std_deviations = [10, 1, 1/(60*60)];
std_deviations = deg2rad(std_deviations);
w_list = 1 ./ (std_deviations .^ 2);
w_list = w_list / sum(w_list);
b_list = [b_moon, b_sun, b_star];
r_list = [r_moon, r_sun, r_star];

% Compute the QUEST polynomial coefficients
[l a b c d] = quest_polynomial(w_list, b_list, r_list);
fprintf('QUEST polynomial cofficients (a, b, c, d) = (%e, %e, %e, %e):\n\n', a, b, c, d)

% Solve the QUEST polynomial for the maximum eigenvalue
psi_quest = @(x) x^4 + a * x^3 + b * x^2 + c * x + d;
dpsi_quest = @(x) 4 * x^3 + 3 * a * x^2 + 2 * b * x + c;
[lambda_max, err, iters] = newton_raphson(psi_quest, dpsi_quest, sum(w_list));
fprintf('The maximum eigenvalue using QUEST: %f\n', lambda_max)
assert(err < 1e-9);
% Verify that it is correct
B = attitude_profile_matrix(w_list, b_list, r_list);
K = davenport_matrix(B);
[V, D] = eig(K);
D = diag(D);
assert(abs(lambda_max - max(D)) < 1e-6)

% Estimate the attitude quaternion
z = K(1:3, 4);
rho = lambda_max + trace(B);
S = B + B';
L = rho * eye(3) - S;
q = inv(L) * z;
q = [q; 1];
q_BR_quest = q / norm(q)

% Compute the attitude error
A_BR_quest = quat2DCM(q_BR_quest);
theta_err_quest = attitude_error(A_BR_quest, true_A_BR_TRIAD);
fprintf('Attitude error relative to TRIAD = %f degrees\n\n', rad2deg(theta_err_quest))

fprintf('Part (b)\n')
sigma_total_squared = 1 / (sum(rad2deg(std_deviations) .^ (-2)));
P = attitude_covariance_matrix(lambda_max, B, A_BR_quest, sigma_total_squared)

fprintf('Part (d)\n')
F = fisher_information_matrix(std_deviations, b_list)
