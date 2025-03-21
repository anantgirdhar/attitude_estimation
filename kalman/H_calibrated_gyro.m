function H = H_calibrated_gyro(A, r_vectors)
  % Compute the measurement sensitivity matrix for calibrated gyro
  % Inputs:
  % - A: 3x3 DCM representing the current state
  % - r_vectors: 3mx1 vector measurements (m = number of sensors)
  % Outputs:
  % - H: 3mx3 measurement estimate

  addpath(genpath('../utils/'))

  m = length(r_vectors) / 3;
  H = zeros(3*m, 3);
  for i = 1:m
    H(3*i-2:3*i, 1:3) = cross_matrix(A * r_vectors(3*i-2:3*i, 1));
  end

end
