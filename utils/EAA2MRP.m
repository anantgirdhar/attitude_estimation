function p = EAA2MRP(e, theta)
  % Convert an Euler Axis/Angle to Modified Rodrigues Parameters
  p = e * tan(theta / 4);
end
