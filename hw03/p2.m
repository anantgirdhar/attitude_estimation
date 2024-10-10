% fprintf('Homework 03 Problem 2\n\n\n')
% Author: Anant Girdhar
% Date: 2024-10-08

set(groot, 'defaultLineLineWidth', 2.0);

% Part (a)
% Plot simple GTSat trajectory

% Set the initial conditions
inclination = 52;  % degrees
altitude = 400e3;  % m
lat0 = 0;
long0 = -90;  % degrees

% Get the position of the satellite over the given time frame
tspan = 0:60:16200;
latitudes = zeros(size(tspan));
longitudes = zeros(size(tspan));
for i = 1:size(tspan, 2)
  [latitudes(i), longitudes(i)] = issorb(tspan(i));
end

latitudes = rad2deg(latitudes);
longitudes = rad2deg(longitudes);

% Plot the coordinates on an outline plot
load topo topo topomap1;
figure('Position', [0, 0, 1400, 700])
contour(0:359, -89:90, topo, [0 0])
axis equal
box on
ax = gca;
ax.XLim = [0 360];
ax.YLim = [-90 90];
ax.XTick = [0 60 120 180 240 300 360];
ax.YTick = [-90 -60 -30 0 30 60 90];
hold on
plot(longitudes, latitudes, 'r.')
hold off
saveas(gcf, 'p2_trajectory.jpg')


% Part (b)
% Model the Earth's magnetic field vector at the satellite locations

% Compute the magnetic field
B_ned = zeros(size(tspan, 2), 3);
day_stamp_0 = datenum('October 1, 2024');
for i = 1:size(tspan, 2)
  day_stamp = day_stamp_0 + floor(tspan(i) / (24 * 60 * 60));
  B_ned(i, :) = igrf(day_stamp, latitudes(i), longitudes(i), altitude / 1000, 'geodetic');
end

% Visualize the B-field
figure('Position', [0, 0, 1400, 900])
subplot(311);
plot(tspan, B_ned(:, 1))
ylabel('Bx\_ned (nT)')
set(gca, 'fontsize', 18)
title('Components of B-field in the NED frame - Anant Girdhar')
subplot(312);
plot(tspan, B_ned(:, 2))
ylabel('By\_ned (nT)')
set(gca, 'fontsize', 18)
subplot(313);
plot(tspan, B_ned(:, 3))
ylabel('Bz\_ned (nT)')
xlabel('time (s)')
set(gca, 'fontsize', 18)
saveas(gcf, 'p2_bfield_ned.jpg')

figure('Position', [0, 0, 1400, 400])
plot(tspan, vecnorm(B_ned'))
ylabel('B\_ned (nT)')
xlabel('time (s)')
title('Magnitude of B-field in the NED frame - Anant Girdhar')
saveas(gcf, 'p2_bmagnitude_ned.jpg')

% Part (c)
% Convert the B_ned vectors to B_eci
B_eci = zeros(size(tspan, 2), 3);
for i = 1:size(tspan, 2)
  B_eci(i, :) = ned2eci(B_ned(i, :)', latitudes(i), longitudes(i), tspan(i))';
end

% Visualize the B-field
figure('Position', [0, 0, 1400, 900])
subplot(311);
plot(tspan, B_eci(:, 1))
ylabel('Bx\_eci (nT)')
set(gca, 'fontsize', 18)
title('Components of B-field in the ECI frame - Anant Girdhar')
subplot(312);
plot(tspan, B_eci(:, 2))
ylabel('By\_eci (nT)')
set(gca, 'fontsize', 18)
subplot(313);
plot(tspan, B_eci(:, 3))
ylabel('Bz\_eci (nT)')
xlabel('time (s)')
set(gca, 'fontsize', 18)
saveas(gcf, 'p2_bfield_eci.jpg')

figure('Position', [0, 0, 1400, 400])
plot(tspan, vecnorm(B_eci'))
ylabel('B\_eci (nT)')
xlabel('time (s)')
title('Magnitude of B-field in the ECI frame - Anant Girdhar')
saveas(gcf, 'p2_bmagnitude_eci.jpg')

close all
