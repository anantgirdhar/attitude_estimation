fprintf('Homework 04 Problem 6\n\n')
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
% Get star vectors
b_star = [1 0 0]';
r_star = quat2DCM(q_BR_st)' * b_star

fprintf('Part (b)\n')
% Get the QUEST polynomial coefficients
w_list = ones(1, 3) / 3;
b_list = [b_moon, b_sun, b_star];
r_list = [r_moon, r_sun, r_star];
[l a b c d] = quest_polynomial(w_list, b_list, r_list);
fprintf('QUEST polynomial cofficients (a, b, c, d) = (%e, %e, %e, %e):\n\n', a, b, c, d);

fprintf('Part (c)\n')
% Compute the maximum eigenvalue
psi_quest = @(x) x^4 + a * x^3 + b * x^2 + c * x + d;
dpsi_quest = @(x) 4 * x^3 + 3 * a * x^2 + 2 * b * x + c;
[lambda_max, err, iters] = newton_raphson(psi_quest, dpsi_quest, sum(w_list));
assert(err < 1e-9);
fprintf('The maximum eigenvalue using QUEST: %f\n\n', lambda_max);
% Verify that it is correct
B = attitude_profile_matrix(w_list, b_list, r_list);
K = davenport_matrix(B);
[V, D] = eig(K);
D = diag(D);
assert(abs(lambda_max - max(D)) < 1e-6)

fprintf('Part (d)\n')
% Estimate the attitude quaternion
z = K(1:3, 4);
rho = lambda_max + trace(B);
S = B + B';
L = rho * eye(3) - S;
% I realize computing the inverse here probably defeats the purpose of this
% method, but I didn't want to write a more efficient way of doing this
q = inv(L) * z;
q = [q; 1];
q_BR_quest = q / norm(q)

fprintf('Part (e)\n')
% Compute the attitude error
A_BR_quest = quat2DCM(q_BR_quest);
theta_err_quest = attitude_error(A_BR_quest, true_A_BR_TRIAD);
fprintf('Attitude error relative to TRIAD = %f degrees\n', rad2deg(theta_err_quest))
