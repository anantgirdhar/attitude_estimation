% fprintf('Homework03 Problem 7\n\n\n')
% Author: Anant Girdhar
% Date: 2024-10-09

set(groot, 'defaultLineLineWidth', 2.0);

% Part (b)
% Draw the stability diagram

zeta = 1 / (12 * pi ^ 2);

k1 = -1:0.0001:1;
k3 = -1:0.0001:1;

[K1, K3] = meshgrid(k1, k3);

% Compute C1
C1 = (K1 > K3);

% Compute C2
lhs = k1;
rhs = (zeta + K3) ./ (1 + zeta * K3);
C2 = (lhs < rhs);
% imshow(lhs > rhs)

% Compute C3
C3 = (K1 .* K3 > 0);

% Compute C4
% Restrict this condition to the parts where K1*K3 > 0
% Otherwise, you get complex numbers that aren't really meaningful
K1_mod = K1;
K3_mod = K3;
K1_mod(K1 > 0 & K3 < 0) = 0;
K3_mod(K1 < 0 & K3 > 0) = 0;
lhs = 1 + 3 * K1_mod + K1_mod .* K3_mod;
rhs = 4 * sqrt(K1_mod .* K3_mod);
C4 = (lhs > rhs);

% Before plotting, remember to flip the image because of how imshow plots things

% First plot each of the individual constraints
imshow(flip(C1)*0.8)
title('Constraints Applied: C1')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C1.jpg')

imshow(flip(C2)*0.8)
title('Constraints Applied: C2')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C2.jpg')

imshow(flip(C3)*0.8)
title('Constraints Applied: C3')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C3.jpg')

imshow(flip(C4)*0.8)
title('Constraints Applied: C4')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C4.jpg')

% Then slowly build up to the full solution
imshow(flip(C4 & C3)*0.8)
title('Constraints Applied: C4 and C3')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C4C3.jpg')

imshow(flip(C4 & C3 & C1)*0.8)
title('Constraints Applied: C4 and C3 and C1')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C4C3C1.jpg')

imshow(flip(C4 & C3 & C1 & C2)*0.8)
title('Constraints Applied: All')
xlabel('k_1')
ylabel('k_3')
saveas(gcf, 'p7_C4C3C1C2.jpg')

close all
