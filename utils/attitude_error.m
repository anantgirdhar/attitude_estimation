function theta = attitude_error(A_estimated, A_true)
  % Estimate the attitude error given estimated and true DCMs
  theta = 2 * asin(norm(A_estimated - A_true, "fro") / sqrt(8));
end
