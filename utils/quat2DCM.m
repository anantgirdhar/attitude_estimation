function A = quat2DCM(q)
  % Convert a quaternion to a Direction Cosine Matrix
  % Compute some quantities that appear over and over again
  q1sq = q(1) ^ 2;
  q2sq = q(2) ^ 2;
  q3sq = q(3) ^ 2;
  q4sq = q(4) ^ 2;
  q1q2 = q(1) * q(2);
  q1q3 = q(1) * q(3);
  q1q4 = q(1) * q(4);
  q2q3 = q(2) * q(3);
  q2q4 = q(2) * q(4);
  q3q4 = q(3) * q(4);
  % Then assemble the matrix entries
  A11 =  q1sq - q2sq - q3sq + q4sq;
  A22 = -q1sq + q2sq - q3sq + q4sq;
  A33 = -q1sq - q2sq + q3sq + q4sq;
  A12 = 2 * (q1q2 + q3q4);
  A21 = 2 * (q1q2 - q3q4);
  A13 = 2 * (q1q3 - q2q4);
  A31 = 2 * (q1q3 + q2q4);
  A23 = 2 * (q2q3 + q1q4);
  A32 = 2 * (q2q3 - q1q4);
  A = [ ...
    A11 A12 A13; ...
    A21 A22 A23; ...
    A31 A32 A33 ...
    ];
end
