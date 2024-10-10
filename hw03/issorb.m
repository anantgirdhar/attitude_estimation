function [latitude, longitude] = issorb(t)

  inclination = deg2rad(52);  % radians
  altitude = 400e3;  % m
  lat0 = 0;  % radians
  long0 = -pi/2;  % radians
  GM = 3.986004418e14;  % m^3 s^-2 (taken from Wikipedia)
  R = 6378137;  % m (taken from ecef2lla function body)

  omega_orbital = sqrt(GM / (R + altitude)^3);

  Theta = omega_orbital * t;
  Theta_0 = pi / 2;
  position_eci = [
    (R + altitude) * cos(inclination) * cos(Theta - Theta_0);
    (R + altitude) * sin(Theta - Theta_0);
    (R + altitude) * sin(inclination) * cos(Theta - Theta_0)];

  position_ecef = eci2ecef(position_eci, t);
  x = position_ecef(1);
  y = position_ecef(2);
  z = position_ecef(3);

  [latitude, longitude, alt] = ecef2lla(x, y, z);
  assert(abs(alt - altitude) < 1e-6);

end

% % Tests: Use the period to verify that after a quarter/half/full period, the spacecraft is where it should beA
% T = 2 * pi * sqrt((h + R)^3 / GM);
% [lat, lon] = issorb(0);
% assert(abs(lat) < 1e-6);
% assert(abs(lon - 3 * pi / 2) < 1e-6);
% [lat, lon] = issorb(T/2);
% assert(abs(lat) < 1e-6);
% assert(abs(lon - pi / 2) < 1e-6);
% [lat, lon] = issorb(T);
% assert(abs(lat) < 1e-6);
% assert(abs(lon - 3 * pi / 2) < 1e-6);
