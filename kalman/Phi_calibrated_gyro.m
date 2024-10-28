function P = Phi_calibrated_gyro(omega_meas, dt)
  % Compute the state transition matrix for a calibrated gyro
  % Inputs
  % - omega_meas: 3x1 rotate rate vector
  % - dt: 1x1 time step
  % Outputs:
  % - P: 3x3 state transition matrix

  addpath(genpath('../utils'))

  P = eye(3) - dt * cross_matrix(omega_meas);
end
