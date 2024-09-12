fprintf('Problem 6\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-11

% Load relevant utilities
addpath(genpath('../utils/'))

set(groot, 'defaultLineLineWidth', 2.0);


e = [1 0 1]';
theta_deg_list = -300:1:300;
theta_list = deg2rad(theta_deg_list);
nthetas = length(theta_list);


fprintf('Part (a)\n\n')
% Compute the MRP and MRP shadow set

p_list = zeros(3, nthetas);  % Modified Rodrigues Parameters
ps_list = zeros(3, nthetas);  % Modified Rodrigues Parameters Shadow Set
for i = 1:nthetas
  p_list(:, i) = EAA2MRP(e, theta_list(i));
  ps_list(:, i) = MRP2sMRP(p_list(:, i));
end


fprintf('Part (b)\n\n')
% Plot elements of MRP
subplot(3, 1, 1);
plot(theta_deg_list, p_list(1, :));
ylabel('p(1)');
set(gca, 'fontsize', 18);
title('Components of the MRP')
subplot(3, 1, 2);
plot(theta_deg_list, p_list(2, :));
ylabel('p(2)');
set(gca, 'fontsize', 18);
subplot(3, 1, 3);
plot(theta_deg_list, p_list(3, :));
ylabel('p(3)');
xlabel('Theta (deg)');
set(gca, 'fontsize', 18);


fprintf('Part (c)\n\n')
% Plot elements of MRP shadow set
figure
subplot(3, 1, 1);
plot(theta_deg_list, ps_list(1, :));
ylabel('ps(1)');
set(gca, 'fontsize', 18);
title('Components of the MRP Shadow Set')
subplot(3, 1, 2);
plot(theta_deg_list, ps_list(2, :));
ylabel('ps(2)');
set(gca, 'fontsize', 18);
subplot(3, 1, 3);
plot(theta_deg_list, ps_list(3, :));
ylabel('ps(3)');
xlabel('Theta (deg)');
set(gca, 'fontsize', 18);


fprintf('Part (d)\n\n')
% Switch between the MRP and shadow set when the norm exceeds 1
p_sw_list = zeros(3, nthetas);  % MRPs with switching
switch_list = zeros(nthetas);
switch_ = 0;  % 0 is MRP and 1 is the shadow set
for i = 1:nthetas
  if switch_ == 0 && norm(p_list(:, i)) > 1
    switch_ = 1;
  elseif switch_ == 1 && norm(ps_list(:, i)) > 1
    switch_ = 0;
  end
  switch_list(i) = switch_;
  if switch_ == 0
      p_sw_list(:, i) = p_list(:, i);
  else
    p_sw_list(:, i) = ps_list(:, i);
  end
end
% Plot the components
figure
subplot(3, 1, 1);
plot(theta_deg_list, p_sw_list(1, :));
ylabel('p_{sw}(1)');
set(gca, 'fontsize', 18);
title('Components of MRP with Switching')
subplot(3, 1, 2);
plot(theta_deg_list, p_sw_list(2, :));
ylabel('p_{sw}(2)');
set(gca, 'fontsize', 18);
subplot(3, 1, 3);
plot(theta_deg_list, p_sw_list(3, :));
ylabel('p_{sw}(3)');
xlabel('Theta (deg)');
set(gca, 'fontsize', 18);

figure
plot(theta_deg_list, switch_list);
ylabel('switch list');
xlabel('Theta (deg)');
set(gca, 'fontsize', 18);
title('MRP (0) vs Shadow Set (1) Chosen')
