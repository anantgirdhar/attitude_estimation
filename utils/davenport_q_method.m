function q_BR = davenport_q_method(w_list, b_list, r_list)
  % Compute the best estimate of the attitude represented as a quaternion,
  % q_BR, using Davenport's q-method
  % Arguments:
  % - w: 1xn vector of weights
  % - b_list: 3xn matrix of B-frame vectors
  % - r_list: 3xn matrix of corresponding R-frame vectors
  B = attitude_profile_matrix(w_list, b_list, r_list);
  K = davenport_matrix(B);
  [V, D] = eig(K);
  % Find the eigenvector corresponding to the maximum eigenvalue
  [max_eigenvalue, max_eigenvalue_index] = max(diag(D));
  q_BR = V(:, max_eigenvalue_index);
end
