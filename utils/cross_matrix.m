function v_cross = cross_matrix(v)
  % Return the cross product matrix corresponding to v
  v_cross = [ ...
        0 -v(3)  v(2); ...
     v(3)     0 -v(1); ...
    -v(2)  v(1)     0];
end
