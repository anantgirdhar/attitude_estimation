function F = fisher_information_matrix(sigmas, b_list)
  % Compute the Fisher Information Matrix
  % Arguments:
  % - sigmas: 1xn array
  %     the standard deviations of the sensors in radians
  % - b_list: 3xn matrix
  %     the B-frame vectors

  F = zeros(3);
  for i = 1:length(sigmas)
    F = F + (eye(3) - b_list(:, i) * b_list(:, i)') ./ (sigmas(i) ^ 2);
  end

end
