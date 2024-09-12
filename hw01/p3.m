fprintf('Problem 3\n\n\n')
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
% Convert a vector in RTN frame to body frame
v_N = [2; 1; -1];
v_K = A_NK' * v_N


fprintf('Part (b)\n\n')
% Convert a vector in body frame to RTN frame
w_K = [-1; -2; 2];
w_N = A_NK * w_K


fprintf('Part (c)\n\n')
% Compute the angle between vectors v and w
% First let's try it in the body frame
theta_K = acos(v_K' * w_K / (norm(v_K) * norm(w_K)));
% Next try it in the RTN frame
theta_N = acos(v_N' * w_N / (norm(v_N) * norm(w_N)));
% These should be the same
assert(theta_K == theta_N)
fprintf('The angle between v and w vectors is %.4f deg\n\n', rad2deg(theta_K))


fprintf('Part (d)\n\n')
nose_direction_K = [0, 1, 0]';
nose_direction_N = A_NK * nose_direction_K;
% The local horizontal plane is formed by the j and k axes in the RTN frame
% Find the projection of the nose vector in this plane
nose_direction_projection_N = nose_direction_N;
nose_direction_projection_N(1) = 0;  % Setting the i-component to zero gives the projection
% Now find the angle between the projection and the nose
Delta = dot(nose_direction_N, nose_direction_projection_N) / ...
  (norm(nose_direction_N) * norm(nose_direction_projection_N));
fprintf('Delta = %.4f deg\n', rad2deg(Delta))
