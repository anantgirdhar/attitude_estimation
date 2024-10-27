function P = attitude_covariance_matrix(lambda_max, B, A, c)
  % Compute the attitude covariance matrix
  % Arguments:
  % - lambda_max: float
  %     the maximum eigenvalue of the Davenport matrix
  % - B: 3x3 matrix
  %     the attitude profile matrix
  % - A: 3x3 matrix
  %     the estimated DCM
  % - c: float
  %     a scaling factor
  %     default: 1

  assert(nargin >= 3);
  if nargin < 4
    c = 1;
  end

  P = c * inv(lambda_max * eye(3) - B * A');

end
