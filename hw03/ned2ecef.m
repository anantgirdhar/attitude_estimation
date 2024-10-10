function ecefvec = ned2ecef(nedvec, latitude, longitude)
  % This function returns the ECEF representation of an NED coordinated vector
  % Arguments:
  % - nedvec: 3x1 vector coordinated in NED frame
  % - latitude: scalar in degrees
  % - longitude: scalar in degrees
  % Outputs:
  % ecefvec: 3x1 vector coordinated in ECEF frame

  % Write out the unit vectors n, e, d in the ecef frame
  n_ecef = [-sind(latitude) * cosd(longitude);
    -sind(latitude) * sind(longitude);
    cosd(latitude)];
  e_ecef = [-sind(longitude);
    cosd(longitude);
    0];
  d_ecef = [-cosd(latitude) * cosd(longitude);
    -cosd(latitude) * sind(longitude);
    -sind(latitude)];

  % The NED to ECEF transformation matrix is just these vectors as the columns of the matrix
  A = [n_ecef, e_ecef, d_ecef];
  ecefvec = A * nedvec;

end
