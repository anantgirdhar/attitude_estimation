function R = R_isotropic_sensors(sigmas)
  % Compute the measurement error covariance matrix assuming isotropic sensors
  % It is assumed that there are m sensors each returning a vector observation
  % Inputs:
  % - sigmas: mx1 list of standard deviations for each sensor

  m = length(sigmas);
  
  % The R matrix is a diagonal matrix
  % First create the values along the diagonal
  % The sensors are isotropic and return 3 component vectors
  % So the block corresponding to each sensor should be a 3x3
  Z = zeros(3*m, m);
  for i = 1:m
    Z((3*i-2):(3*i), i) = 1;
  end
  R = Z * sigmas;

  % Then turn it into a square diagonal matrix
  R = diag(R);

end
