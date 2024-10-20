fprintf('Homework 04 Problem 4\n\n')
% Author: Anant Girdhar
% Date: 2024-10-19

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);

% Given celestial object directions
r_moon = [0.5, 0, -0.5 * sqrt(3)]';
b_moon = [0.8126, -0.0316, 0.5820]';
r_sun = [0, 0, -1]';
b_sun = [0.9999, -0.0101, -0.0101]';

% Load the results from problem 3
load('P3_results.mat')
true_A_BR_TRIAD_1 = estimated_A_BR_TRIAD_moon;
true_A_BR_TRIAD_2 = estimated_A_BR_TRIAD_sun;
clearvars estimated_A_BR_TRIAD_moon estimated_A_BR_TRIAD_sun;

fprintf('Part (a)\n')
% Estimate the DCM using TRIAD using moon vectors first
estimated_A_BR_TRIAD_1 = A_BR_TRIAD(b_moon, b_sun, r_moon, r_sun);
fprintf('If the moon vectors are used first:\n')
disp(estimated_A_BR_TRIAD_1)

fprintf('Part (b)\n')
% Estimate the attitude error
theta_TRIAD_1 = attitude_error(estimated_A_BR_TRIAD_1, true_A_BR_TRIAD_1);
fprintf('Attitude error = %f degrees\n\n', rad2deg(theta_TRIAD_1))

fprintf('Part (c)\n')
% Use the sun vector first instead
estimated_A_BR_TRIAD_2 = A_BR_TRIAD(b_sun, b_moon, r_sun, r_moon);
fprintf('If the sun vectors are used first:\n')
disp(estimated_A_BR_TRIAD_2)
fprintf('\n')
theta_TRIAD_2 = attitude_error(estimated_A_BR_TRIAD_2, true_A_BR_TRIAD_2);
fprintf('Attitude error = %f degrees\n\n', rad2deg(theta_TRIAD_2))

fprintf('Part (d)\n')
% Use Davenport's method with equal weights
w_list = [0.5, 0.5];
b_list = [b_moon, b_sun];
r_list = [r_moon, r_sun];
fprintf('Attitude profile matrix:\n')
B1 = attitude_profile_matrix(w_list, b_list, r_list)
q_BR_davenport_1 = davenport_q_method(w_list, b_list, r_list);
fprintf('Estimated DCM using Davenport''s method:\n')
A_BR_davenport_1 = quat2DCM(q_BR_davenport_1)
theta_davenport_1 = attitude_error(A_BR_davenport_1, true_A_BR_TRIAD_1);
fprintf('Attitude error = %f degrees\n\n', rad2deg(theta_davenport_1))

fprintf('Part (e)\n')
% Weight the sun measurement 10 times as compared to the moon measurement
w_list = [1, 10];
b_list = [b_moon, b_sun];
r_list = [r_moon, r_sun];
fprintf('Attitude profile matrix:\n')
B2 = attitude_profile_matrix(w_list, b_list, r_list)
q_BR_davenport_2 = davenport_q_method(w_list, b_list, r_list);
fprintf('Estimated DCM using Davenport''s method:\n')
A_BR_davenport_2 = quat2DCM(q_BR_davenport_2)
theta_davenport_2 = attitude_error(A_BR_davenport_2, true_A_BR_TRIAD_2);
fprintf('Attitude error = %f degrees\n', rad2deg(theta_davenport_2))
