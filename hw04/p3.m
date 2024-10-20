fprintf('Homework 04 Problem 3\n\n')
% Author: Anant Girdhar
% Date: 2024-10-19

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Given celestial object directions
r_moon = [0.5, 0, -0.5 * sqrt(3)]';
b_moon = [0.5 * sqrt(3), 0, 0.5]';
r_sun = [0, 0, -1]';
b_sun = [1, 0, 0]';

fprintf('Part (a)\n\n')
% Estimate the DCM using TRIAD
% It matters which vector we use first so let's try it both ways
estimated_A_BR_TRIAD_moon = A_BR_TRIAD(b_moon, b_sun, r_moon, r_sun);
estimated_A_BR_TRIAD_sun = A_BR_TRIAD(b_sun, b_moon, r_sun, r_moon);
fprintf('The direction cosine matrix estimated using TRIAD depends on the order of the vectors\n')
fprintf('So let''s try it both ways\n')
fprintf('If the moon vectosr are used first (the way the problem lists it):\n')
disp(estimated_A_BR_TRIAD_moon)
fprintf('If the sun vectors are used first:\n')
disp(estimated_A_BR_TRIAD_sun)
fprintf('\n')

fprintf('Part (b)\n')
% Compute the attitude profile matrix
b_list = [b_moon, b_sun];
r_list = [r_moon, r_sun];
w_list = [0.5, 0.5];
B = attitude_profile_matrix(w_list, b_list, r_list)
fprintf('\n')

fprintf('Part (c)\n')
% Compute the Davenport Matrix, K(B)
K = davenport_matrix(B)
fprintf('\n')

fprintf('Part (d)\n')
% Estimate the attitude quaternion using Davenport's method
q_BR_davenport = davenport_q_method(w_list, b_list, r_list)
fprintf('\n')

fprintf('Part (e)\n')
% Express the Davenport attitude as a DCM
A_BR_davenport = quat2DCM(q_BR_davenport)

% Save some variables to use them in future problems
save('P3_results.mat', 'estimated_A_BR_TRIAD_moon', 'estimated_A_BR_TRIAD_sun');
