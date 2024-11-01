function [qm, Pm] = propagate_attitude(qp, omega_hat, Pp, Phi, Y, Q, dt)
  % Perform the propagation step for the attitude problem.
  % Reminder that even though this is computing the quaternion, the "state" for
  % the Kalman filter is delta x (which may include states in addition to the
  % small angle error/update). 'n' is the number of state components.
  % Inputs:
  % - qp: 4x1 quaternion after measurement
  % - omega_hat: 3x1 corrected rotation rate (corrected for bias)
  % - Pp: nxn state covariance matrix after measurement
  % - Phi: nxn state transition matrix
  % - Y: nxn process noise "gain" matrix
  % - Q: nxn process noise covariance atrix
  % - dt: 1x1 time step
  % Outputs:
  % - qm: 4x1 quaternion
  % - Pm: nxn state covariance matrix
  qm = Theta(omega_hat, dt) * qp;
  Pm = Phi * Pp * Phi' + Y * Q * Y';
end
