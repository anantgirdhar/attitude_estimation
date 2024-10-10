function ecivec = ecef2eci(ecefvec, t)
  % This function returns the ECI representation of an ECEF coordinated vector
  % It assumes that the ECI frame and ECEF frame are aligned at t = 0
  % Arguments:
  % - ecefvec: 3x1 vector coordinated in ECEF frame
  % - t: scalar time in seconds
  % Outputs:
  % ecivec: 3x1 vector coordinated in ECI frame

  % This is just a rotation about the z axis by an angle Theta defined by how
  % much the earth has rotated in the given time
  Theta = 2 * pi * t / (24 * 60 * 60);
  A = [cos(Theta), sin(Theta), 0;
    -sin(Theta), cos(Theta), 0;
    0, 0, 1]';
  ecivec = A * ecefvec;

end
