function ps = MRP2sMRP(p)
  % Convert Modified Rodrigues Parameters to the shadow set
  ps = - p / (norm(p) ^ 2);
end
