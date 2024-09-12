fprintf('Problem 4\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-11

% Load relevant utilities
addpath(genpath('../utils/'))


a = deg2rad(26.56);
b = deg2rad(41.81);
g = deg2rad(26.56);

% Create the rotation matrix going from the body frame (K) to the RTN frame (N)
A_NK = rotation_matrix_123(a, b, g);


fprintf('Part (a)\n\n')
% Find the axis and angle of rotation
[e, theta] = DCM2EAA(A_NK);
fprintf('rotation axis vector, e = (%s)\n', sprintf(' %d ', e))
fprintf('theta = %.2f deg\n\n', theta)


fprintf('\nPart (b)\n\n')
% Demonstrate that e is the eigenvector for A_NK
fprintf('A_NK * e = (%s)\n', sprintf(' %d ', A_NK * e))
disp('Since this matches exactly with e, it is an eigenvector with eigenvalue 1')


fprintf('\n\nPart (c)\n\n')
% Convert this to the Euler parameter
q_NK = EAA2quat(e, theta);
fprintf('q_NK = (%s)\n', sprintf(' %d ', q_NK))


fprintf('\n\nPart (d)\n\n')
% Compute the four four-component vectors directly from the DCM
q_NK_unnormalized = [ ...
  DCM2quat(A_NK, 1, 1) ...
  DCM2quat(A_NK, 2, 1) ...
  DCM2quat(A_NK, 3, 1) ...
  DCM2quat(A_NK, 4, 1) ...
  ];
fprintf('q_NK (unnormalized 1) = (%s)\n', sprintf(' %8.4d ', q_NK_unnormalized(:, 1)))
fprintf('q_NK (unnormalized 2) = (%s)\n', sprintf(' %8.4d ', q_NK_unnormalized(:, 2)))
fprintf('q_NK (unnormalized 3) = (%s)\n', sprintf(' %8.4d ', q_NK_unnormalized(:, 3)))
fprintf('q_NK (unnormalized 4) = (%s)\n', sprintf(' %8.4d ', q_NK_unnormalized(:, 4)))
norms = [ ...
  norm(q_NK_unnormalized(:, 1)) ...
  norm(q_NK_unnormalized(:, 2)) ...
  norm(q_NK_unnormalized(:, 3)) ...
  norm(q_NK_unnormalized(:, 4)) ...
  ];
[~, max_norm_index] = max(norms);
q_NK_from_DCM = ...
  q_NK_unnormalized(:, max_norm_index) ...
  / norm(q_NK_unnormalized(:, max_norm_index));
fprintf('\nq_NK (from DCM) = (%s)\n', sprintf(' %d ', q_NK_from_DCM))
assert(q_NK_from_DCM == q_NK)
disp('Which is the same as what we obtained in part (c)')


fprintf('\n\nPart (d)\n\n')
% Verify that q_NK has unit norm
fprintf('norm(q_NK) = %d\n', norm(q_NK))
