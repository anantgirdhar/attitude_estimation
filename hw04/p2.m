fprintf('Homework 04 Problem 2\n\n')
% Author: Anant Girdhar
% Date: 2024-10-20

% Given the total system torque
Lw = [1, -1, 1]';  % N.m

% Find the individual wheel torques and verify that it works

fprintf('Part (a)\n\n')
% Using the NASA standard
alpha = 1 / sqrt(3);
beta = 1 / sqrt(3);
gamma = 1 / sqrt(3);
W4_NASA = [...
  1 0 0 alpha; ...
  0 1 0 beta; ...
  0 0 1 gamma ...
  ];
W4_NASA_plus = W4_NASA' * inv(W4_NASA * W4_NASA');
Lwi = W4_NASA_plus * Lw
Lw_NASA = W4_NASA * Lwi
assert(all(Lw_NASA - Lw < 1e-9))

fprintf('Part (b)\n\n')
% Using the four wheel pyramid
a = 1 / sqrt(2);
b = 1 / sqrt(2);
c = sqrt(3) / 2;
d = 0.5;
W4_FWP = [...
  a -a 0 0; ...
  b b c c; ...
  0 0 d -d ...
  ];
W4_FWP_plus = W4_FWP' * inv(W4_FWP * W4_FWP');
Lwi = W4_FWP_plus * Lw
Lw_FWP = W4_FWP * Lwi
assert(all(Lw_FWP - Lw < 1e-9))

fprintf('Part (c)\n\n')
% Using the six wheel pyramid (James Webb Space Telescope)
a = 1 / sqrt(2);
b = 1 / sqrt(2);
c = 1 / sqrt(2);
d = 1 / sqrt(2);
W4_JWST = [...
  b, c, b, c, b, c; ...
  0, sqrt(3)*d/2, sqrt(3)*a/2, 0, -sqrt(3)*a/2, -sqrt(3)*d/2; ...
  a, d/2, -a/2, -d, -a/2, d/2 ...
  ];
W4_JWST_plus = W4_JWST' * inv(W4_JWST * W4_JWST');
Lwi = W4_JWST_plus * Lw
Lw_JWST = W4_JWST * Lwi
assert(all(Lw_JWST - Lw < 1e-9))
