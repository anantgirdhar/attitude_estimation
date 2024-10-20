function A = A_BR_TRIAD(b1, b2, r1, r2)
  % Compute the TRIAD solution given vectors b1 and b2 in the B frame and r1
  % and r2 in the R frame
  rx = cross(r1, r2);
  rx = rx / sqrt(rx' * rx);
  bx = cross(b1, b2);
  bx = bx / sqrt(bx' * bx);
  A = b1 * r1' + bx * rx' + cross(b1, bx) * cross(r1, rx)';
end
