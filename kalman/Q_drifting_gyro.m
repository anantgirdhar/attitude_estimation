function Q = Q_drifting_gyro(sigma_v, sigma_u, dt)
  % Compute the process noise covariance matrix assuming isotropic distribution
  a = (sigma_v ^ 2) * dt + (sigma_u ^ 2) * (dt ^ 3) / 3;
  b = - 0.5 * (sigma_u ^ 2) * (dt ^ 2);
  d = (sigma_u ^ 2) * dt;
  Q = [ ...
    a * eye(3), b * eye(3); ...
    b * eye(3), d * eye(3) ...
    ];
end
