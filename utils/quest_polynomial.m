function [l a b c d] = quest_polynomial(w_list, b_list, r_list)
  % Compute the quest polynomial coefficients
  % Arguments:
  % - w: 1xn vector of weights
  % - b_list: 3xn matrix of B-frame vectors
  % - r_list: 3xn matrix of corresponding R-frame vectors
  B = attitude_profile_matrix(w_list, b_list, r_list);
  K = davenport_matrix(B);
  S = B + B';
  z = [ ...
    B(2, 3) - B(3, 2); ...
    B(3, 1) - B(1, 3); ...
    B(1, 2) - B(2, 1) ...
    ];
  % Compute some of the quantities that show up in the characteristic polynomial
  trB = trace(B);
  Ds = det(S);
  adjS = Ds * inv(S);
  trA = trace(adjS);
  z2 = z' * z;
  P1 = z' * S * z;
  P2 = z' * S * S * z;
  % Finally compute the coefficients
  l = 1;
  a = 0;
  b = -2 * trB^2 - z2 + trA;
  c = -P1 - Ds;
  d = trB^4 + trB^2 * z2 - trB^2 * trA - trA * z2 + trB * P1 + trB * Ds - P2;
end
