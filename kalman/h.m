function h = h(A, r_vectors)
  % Compute the estimates of the measurements
  % Inputs:
  % - A: 3x3 DCM representing the current state
  % - r_vectors: 3mx1 vector measurements (m = number of sensors)
  % Outputs:
  % - h: 3mx1 measurement estimate

  m = length(r_vectors) / 3;
  h = r_vectors;
  for i = 1:m
    h(3*i-2:3*i, 1) = A * h(3*i-2:3*i, 1);
  end

end
