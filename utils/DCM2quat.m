function q = DCM2quat(M, variant, unnormalized)
  % Convert Direction Cosine Matrix to quaternion / Euler-Rodrigues parameters
  % variant is an optional argument taking values 1 - 4 that, when specified,
  % forces computation of the quaternion using a specific formulation
  % unnormalized is an optional argument taken values 0 (false) or 1 (true)
  % that, when specified, returns the normalized or unnormalized quaternions
  if ~exist('unnormalized', 'var')
    unnormalized = 0;
  end
  if exist('variant', 'var')
    switch variant
      case 1
        q = DCM2quat_partial_1(M);
      case 2
        q = DCM2quat_partial_2(M);
      case 3
        q = DCM2quat_partial_3(M);
      case 4
        q = DCM2quat_partial_4(M);
    end
    if ~unnormalized
      q = q / norm(q);
    end
  else % if variant is not specified
    q = DCM2quat_partial_1(M);
    if q(1) ~= 0
      q = q / norm(q);
      return
    end
    q = DCM2quat_partial_2(M);
    if q(2) ~= 0
      q = q / norm(q);
      return
    end
    q = DCM2quat_partial_3(M);
    if q(3) ~= 0
      q = q / norm(q);
      return
    end
    q = DCM2quat_partial_4(M);
    if q(4) ~= 0
      q = q / norm(q);
      return
    end
  end % if variant is specified
end

function qp = DCM2quat_partial_1(M)
  % Convert DCM to unnormalized quaternion using variant 1
  trM = trace(M);
  q1 = 1 + 2 * M(1, 1) - trM;
  q2 = M(1, 2) + M(2, 1);
  q3 = M(1, 3) + M(3, 1);
  q4 = M(2, 3) - M(3, 2);
  qp = [ q1 q2 q3 q4 ]';
end

function qp = DCM2quat_partial_2(M)
  % Convert DCM to unnormalized quaternion using variant 2
  trM = trace(M);
  q1 = M(2, 1) + M(1, 2);
  q2 = 1 + 2 * M(2, 2) - trM;
  q3 = M(2, 3) + M(3, 2);
  q4 = M(3, 1) - M(1, 3);
  qp = [ q1 q2 q3 q4 ]';
end

function qp = DCM2quat_partial_3(M)
  % Convert DCM to unnormalized quaternion using variant 3
  trM = trace(M);
  q1 = M(3, 1) + M(1, 3);
  q2 = M(3, 2) + M(2, 3);
  q3 = 1 + 2 * M(3, 3) - trM;
  q4 = M(1, 2) - M(2, 1);
  qp = [ q1 q2 q3 q4 ]';
end

function qp = DCM2quat_partial_4(M)
  % Convert DCM to unnormalized quaternion using variant 4
  trM = trace(M);
  q1 = M(2, 3) - M(3, 2);
  q2 = M(3, 1) - M(1, 3);
  q3 = M(1, 2) - M(2, 1);
  q4 = 1 + trM;
  qp = [ q1 q2 q3 q4 ]';
end

