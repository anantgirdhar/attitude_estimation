function P = Phi_drifting_gyro(omega_hat, dt)
  % Compute the state transition matrix for a drifting gyro
  % Inputs
  % - omega_hat: 3x1 rotate rate vector
  % - dt: 1x1 time step
  % Outputs:
  % - P: 3x3 state transition matrix

  addpath(genpath('../utils'))

  wx = cross_matrix(omega_hat);
  speed = vecnorm(omega_hat);
  s = sin(speed * dt);
  c = cos(speed * dt);

  p11 = eye(3) - wx * s / speed + (wx ^ 2) * (1 - c) / (speed ^ 2);
  p12 = wx * (1 - c) / (speed ^ 2) - eye(3) * dt - (wx ^ 2) * (speed * dt - s) / (speed ^ 3);
  p21 = zeros(3, 3);
  p22 = eye(3);
  
  P = [p11, p12; p21, p22];

end
