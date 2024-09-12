function p = quat2MRP(q)
  % Convert a quaternion to Modified Rodrigues Parameters
  p = q(1:3) / (1 + q(4));
end
