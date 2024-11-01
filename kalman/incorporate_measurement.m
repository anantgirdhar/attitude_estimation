function [qp, Pp, dx_p] = incorporate_measurement(qm, dx_m, y, hm, Hm, Pm, R)
  % Perform the measurement update step for the MEKF.
  % It is assumed that there are n Kalman state variables and m sensor
  % measurement variables.
  % Inputs:
  % - qm: 4x1 quaternion prior to measurement update
  % - dx_m: nx1 Kalman state vector before measurement
  % - y: mx1 actual measurement vector
  % - hm: mx1 estimated measurement vector
  % - Hm: mxn measurement sensitivity matrix
  % - Pm: nxn state covariance matrix before measurement
  % - R: mxm sensor error covariance matrix
  % Outputs:
  % - qp: 4x1 quaternion after measurement update
  % - Pp: nxn state covariance matrix after measurement
  % - dx_p: nx1 Kalman state vector after measurement

  addpath(genpath('../utils'))

  n = length(dx_m);
  I = eye(n);

  K = kalman_gain(Hm, Pm, R);
  dx_p = dx_m + K * (y - hm);
  Pp = (I - K * Hm) * Pm;

  dtheta_p = dx_p(1:3, 1);
  qstar = qm + 0.5 * Xi(qm) * dtheta_p;
  qp = qstar / vecnorm(qstar);

end
