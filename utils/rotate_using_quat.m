function x_rotated = rotate_using_quat(x, q)
  % Rotate a 3x3 vector x using the quaternion q
  Xi = [q(4) * eye(3) + cross_matrix(q(1:3)) ; -q(1:3)'];
  Psi = [q(4) * eye(3) - cross_matrix(q(1:3)) ; -q(1:3)'];
  x_rotated = Xi' * Psi * x;
end
