function Q = Q_calibrated_gyro(sigma_v, dt)
  % Compute the process noise covariance matrix assuming isotropic distribution
  % and no scaling
  Q = dt * (sigma_v ^ 2) * eye(3);
end
