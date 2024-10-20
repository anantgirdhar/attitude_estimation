function B = attitude_profile_matrix(w, b_list, r_list)
  % Compute the attitude profile matrix B
  % Arguments:
  % - w: 1xn vector of weights
  % - b_list: 3xn matrix of B-frame vectors
  % - r_list: 3xn matrix of corresponding R-frame vectors
  num_vectors = size(b_list, 2);
  assert(size(b_list, 1) == 3);
  assert(size(r_list, 2) == num_vectors);
  assert(size(r_list, 1) == 3);
  assert(length(w) == num_vectors);
  B = zeros(3, 3);
  for i = 1:num_vectors
    B = B + w(i) * b_list(:, i) * r_list(:, i)';
  end
end
