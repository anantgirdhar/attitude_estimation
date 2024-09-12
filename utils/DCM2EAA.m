function [e, theta] = DCM2EAA(M)
  % Convert a Direction Cosine Matrix to the corresponding Euler Axis/Angle
  theta = acos((trace(M) - 1) * 0.5);
  e = (0.5 / sin(theta)) * [...
    M(2, 3) - M(3, 2);
    M(3, 1) - M(1, 3);
    M(1, 2) - M(2, 1) ...
    ];
  e = e / norm(e);
end
