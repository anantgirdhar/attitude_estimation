function q = EAA2quat(e, theta)
  % Convert an Euler Axis/Angle to quaternion / Euler-Rodrigues parameters
  q = [e * sin(theta/2); cos(theta/2)];
end
