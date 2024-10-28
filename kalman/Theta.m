function T = Theta(omega_meas, dt)
  % Compute the quaternion update matrix
  % Inputs:
  % - omega_meas: 3x1 measured rotate rate vector
  % - dt: 1x1 sample rate
  % Outputs:
  % - T: 4x4 quaternion update matrix

  addpath(genpath('../utils'))

  % Compute the pieces of T
  speed = vecnorm(omega_meas);
  psi = sin(0.5 * speed * dt) * omega_meas / speed;
  c = cos(0.5 * speed * dt);
  A = c * eye(3) - cross_matrix(psi);

  % Assemble T
  T = [A , psi];
  T = [T ; -psi', c];

end
