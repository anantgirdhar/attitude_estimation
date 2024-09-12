fprintf('Problem 5\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-11

% Load relevant utilities
addpath(genpath('../utils/'))


e = [1 0 1]';
theta_deg_list = -180:1:180;
theta_list = deg2rad(theta_deg_list);
nthetas = length(theta_list);


% fprintf('Part (a)\n\n')
% Compute a bunch of things and make a plot

q_list = zeros(4, nthetas);  % Euler parameters
g_list = zeros(3, nthetas);  % Rodrigues Parameters
p_list = zeros(3, nthetas);  % Modified Rodrigues Parameters
ps_list = zeros(3, nthetas);  % Shadow set of Modified Rodrigues Parameters
for i = 1:nthetas
  q_list(:, i) = EAA2quat(e, theta_list(i));
  g_list(:, i) = quat2RP(q_list(:, i));
  p_list(:, i) = quat2MRP(q_list(:, i));
  ps_list(:, i) = MRP2sMRP(p_list(:, i));
end

set(groot, 'defaultLineLineWidth', 2.0);

plot(theta_deg_list, q_list(3, :));
hold on;
plot(theta_deg_list, g_list(3, :));
plot(theta_deg_list, p_list(3, :));
plot(theta_deg_list, ps_list(3, :));
xlim([-180, 180]);
ylim([-2, 2]);
xlabel('Theta (deg)');
ylabel('Parameter');
legend('q(3)', 'g(3)', 'p(3)', 'ps(3)')

set(gca, 'fontsize', 18);
