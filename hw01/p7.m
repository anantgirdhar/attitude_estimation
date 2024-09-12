fprintf('Problem 7\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-11

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);


e = [1 1 1]';
theta_deg = 180;
theta = deg2rad(theta_deg);


fprintf('Part (a)\n\n')
% Find the MRP, p(e, theta)
p = EAA2MRP(e, theta);
fprintf('p = (%s)\n', sprintf(' %d ', p))


fprintf('\n\nPart (b)\n\n')
% Find the equivalent DCM using the Cayley transform
A = MRP2DCM(p)


fprintf('\n\nPart (c)\n\n')
% Find the equivalent DCM using the Cayley transform
A_DB = A * A


fprintf('\n\nPart (d)\n\n')
% Find composite MRP
num = (1 - norm(p) ^ 2) * p + (1 - norm(p) ^ 2) * p - 2 * cross(p, p);
den = 1 + (norm(p) * norm(p)) ^ 2 - 2 * p' * p;
p_DB = num / den;
fprintf('p_DB = (%s)\n\n', sprintf(' %d ', p_DB))
fprintf('Converting this to the equivalent DCM for verification\n\n')
A_DB_converted = MRP2DCM(p_DB)
disp('which is the same as that in Part (c)')
