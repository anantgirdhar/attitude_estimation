function ecefvec = eci2ecef(ecivec, t)
  % This function returns the ECEF representation of an ECI coordinated vector
  % It assumes that the ECI frame and ECEF frame are aligned at t = 0
  % Arguments:
  % - ecivec: 3x1 vector coordinated in ECI frame
  % - t: scalar time in seconds
  % Outputs:
  % ecefvec: 3x1 vector coordinated in ECEF frame

  % This is just a rotation about the z axis by an angle Theta defined by how
  % much the earth has rotated in the given time
  Theta = 2 * pi * t / (24 * 60 * 60);
  A = [cos(Theta), sin(Theta), 0;
    -sin(Theta), cos(Theta), 0;
    0, 0, 1];
  ecefvec = A * ecivec;

end
