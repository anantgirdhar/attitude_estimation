fprintf('Problem 7\n\n')
% Author: Anant Girdhar
% Date: 2024-09-24

% Load utilities
addpath(genpath('../utils/'));

load A2P7.mat;
% Provides q_SE, B_E, b_S

% Part (a)
fprintf('Part (a)\n\n')
% Compute B_s
B_S = zeros(size(B_E));
for i = 1 : size(B_E, 2)
  B_S(:, i) = rotate_using_quat(B_E(:, i), q_SE(:, i));
end
disp('B_E(:, 1)')
disp(B_E(:, 1))
disp('B_S(:, 1)')
disp(B_S(:, 1))

% Part (b)
fprintf('\nPart (b)\n\n')
% Compute measurement residual
db_S = b_S - B_S;
disp('\Delta b_S(:, 1)')
disp(db_S(:, 1))

% Part (c)
disp('Part (c)')
% Solve for the calibration parameters
% We want to perform regression given the multiple observations
% We can absorb the bias term into the M matrix (call this b0M)
% To make the equation consistent with b0M, we need to add a row of 1s above the B_S "matrix"
B_S_ones = [ones(1, size(B_S, 2)); B_S];
% To get it into the form we want, we need to do a bunch of transposing before solving
b0M = (B_S_ones' \ db_S')';
% Finally, extract the components
b0 = b0M(:, 1)
M = b0M(:, 2:4)

% Part (d)
fprintf('\nPart (d)\n\n')
% Apply the model to the magnetometer measurements
b_S_corrected = (eye(3) + M) ^ (-1) * (b_S - b0);
db_S_corrected = b_S_corrected - B_S;
disp('b_S^C(:, 1)')
disp(b_S_corrected(:, 1))
disp('\Delta b_S^C(:, 1)')
disp(db_S_corrected(:, 1))
