function K = kalman_gain(Hm, Pm, R)
  % Compute the Kalman gain matrix
  % Inputs:
  % - Hm: mxn measurement sensitivity matrix
  % - Pm: nxn state covariance matrix before measurement
  % - R: mxm sensor error covariance matrix
  % Outputs:
  % - K: nxm Kalman gain matrix
  K = Pm * Hm' * inv(Hm * Pm * Hm' + R);
end
