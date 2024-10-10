function ecivec = ned2eci(nedvec, latitude, longitude, t)
  % This function returns the ECI representation of an NED coordinated vector
  % Arguments:
  % - nedvec: 3x1 vector coordinated in NED frame
  % - latitude: scalar in degrees
  % - longitude: scalar in degrees
  % - t: scalar time in seconds
  % Outputs:
  % - ecivec: 3x1 vector coordinated in ECI frame

  ecefvec = ned2ecef(nedvec, latitude, longitude);
  ecivec = ecef2eci(ecefvec, t);

end
