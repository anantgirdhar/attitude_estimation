function M = rotation_matrix_123(a, b, g)
  % a, b, and g are the rotation angles alpha, beta, gamma
  M = [ ...
    cos(b) * cos(g) ...
    cos(a) * sin(g) + sin(a) * sin(b) * cos(g) ...
    sin(a) * sin(g) - cos(a) * sin(b) * cos(g);
    -cos(b) * sin(g) ...
    cos(a) * cos(g) - sin(a) * sin(b) * sin(g) ...
    sin(a) * cos(g) + cos(a) * sin(b) * sin(g);
    sin(b) ...
    -sin(a) * cos(b) ...
    cos(a) * cos(b) ...
    ];
end
