function g = quat2RP(q)
  % Convert a quaternion to Rodrigues Parameters / Gibbs vector
  g = q(1:3) / q(4);
end
