function A = MRP2DCM(p)
  % Convert Modified Rodrigues Parameters to a Direction Cosine Matrix
  vx = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];  % cross product matrix
  A = (eye(3) - vx) * inv(eye(3) + vx);
  A = A ^ 2;
end
