fprintf('Problem 4\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-24

% Cross reference star locations with catalog information

% Load the computed star locations
load A2P3_computed.mat;
image_stars = star_vectors;
image_degrees = angles_degrees;

% Load the catalog
catalog = [...
   0.0262  0.0011 0.9997;
  -0.0312  0.0599 0.9977;
   0.0224 -0.0759 0.9969;
   0.0354  0.0652 0.9972 ];

% Identify the stars
% To do this:
% - Pick a triangle of stars from the image
% - Compare this triangle to all possible triangles in the catalog
% - Once a match has been found, add a fourth star to confirm

% Compute the angles between star pairs in the catalog
catalog = catalog ./ vecnorm(catalog')';
catalog_degrees = acosd(catalog * catalog');

[ci, cj, ck, cw, si, sj, sk, sw] = match_with_catalog(image_degrees, catalog_degrees);
matched_stars = zeros(4, 2);
matched_stars(ci, :) = stars(si, :);
matched_stars(cj, :) = stars(sj, :);
matched_stars(ck, :) = stars(sk, :);
matched_stars(cw, :) = stars(sw, :);
catalog_star_names = {'alpha'; 'beta'; 'gamma'; 'delta'};
fprintf('Stars matched\n')
for i = 1:4
  fprintf('%s ~ (%s)\n', catalog_star_names{i}, sprintf(' %.4f ', matched_stars(i, :)))
end

% Redraw the image with the stars labelled
figure;
image(cleared_image);
colormap(parula(max(max(cleared_image))));
colorbar;
xlabel('u');
ylabel('v');
% Annotate the plot
for i = 1:4
  axPos = get(gca, 'Position');
  xminmax = xlim;
  yminmax = ylim;
  % Take the (u, v) coordinates of each star
  % Add 0.5 because the figure's u-v coordinates are shifted by half a unit
  % relative to the star centroids
  % Then subtract 0.1 to account for the thickness of the text (the left edge
  % of the text is placed at the given coordinates but I want it to be slightly
  % more centered)
  xPlot = matched_stars(i,1) + 0.5 - 0.1;
  yPlot = matched_stars(i,2) + 0.5 - 0.1;
  text(xPlot, yPlot, strcat('\', catalog_star_names{i}), 'FontSize', 18)
end


function [ci, cj, ck, cw, si, sj, sk, sw] = match_with_catalog(image_degrees, catalog_degrees)
  for si = 1:length(image_degrees)
    for sj = (si+1):length(image_degrees)
      for sk = (sj+1):length(image_degrees)
        candidate_angles = [image_degrees(si, sj), image_degrees(sj, sk), image_degrees(sk, si)];
        % fprintf('Candidate: %d %d %d - (%s)\n', si, sj, sk, sprintf(' %d ', candidate_angles))
        [ci, cj, ck, cw, sw] = compare_with_catalog(catalog_degrees, image_degrees, si, sj, sk);
        if ~(ci == 0 && cj == 0 && ck == 0 && cw == 0 && sw == 0)
          return
        end
      end
    end
  end
  ci = 0;
  cj = 0;
  ck = 0;
  cw = 0;
  si = 0;
  sj = 0;
  sk = 0;
  sw = 0;
end


function [ci, cj, ck, cw, sw] = compare_with_catalog(catalog_angles, image_angles, si, sj, sk)
  threshold = 1;  % threshold for assuming that a distance difference is small enough
  image_triangle_angles = [image_angles(si, sj), image_angles(sj, sk), image_angles(sk, si)];
  for ci = 1:length(catalog_angles)
    for cj = 1:length(catalog_angles)
      if cj == ci
        continue
      end
      for ck = 1:length(catalog_angles)
        if ck == ci || ck == cj
          continue
        end
        catalog_triangle_angles = [catalog_angles(ci, cj), catalog_angles(cj, ck), catalog_angles(ck, ci)];
        difference = vecnorm(catalog_triangle_angles - image_triangle_angles);
        if difference < threshold
          % fprintf('           %d %d %d - (%s) - %f\n', ci, cj, ck, sprintf(' %d ', catalog_triangle_angles), difference)
          % There is a possible match here
          % Add a fourth star and compare the 3 additional angles
          for cw = 1:length(catalog_angles)
            if cw == ci || cw == cj || cw == ck
              continue
            end
            additional_catalog_angles = [catalog_angles(cw, ci), catalog_angles(cw, cj), catalog_angles(cw, ck)];
            for sw = 1:length(image_angles)
              if sw == si || sw == sj || sw == sk
                continue
              end
              additional_image_angles = [image_angles(sw, si), image_angles(sw, sj), image_angles(sw, sk)];
              difference = vecnorm(additional_catalog_angles - additional_image_angles);
              if difference < threshold
                % fprintf('           %d %d - %f\n', cw, sw, difference)
                return
              end
            end
          end
        end
      end
    end
  end
  ci = 0;
  cj = 0;
  ck = 0;
  cw = 0;
  sw = 0;
end

% % Part (c)
% disp('Part (c)')
% % Find line of sight vectors for each star
% star_vectors = stars;
% f = 60;
% u_p = 6;
% v_p = 6;
% % First do an origin shift in the sensor plane relative to (u_p, v_p)
% star_vectors(:, 1) = star_vectors(:, 1) - u_p;
% star_vectors(:, 2) = star_vectors(:, 2) - v_p;
% star_vectors(:, 3) = f;
% % Now normalize each row of vectors
% star_vectors = star_vectors ./ vecnorm(star_vectors')';
% disp('Line of sight unit vectors computed:')
% disp(star_vectors')
% fprintf('\n')


% % Part (d)
% % Compute angles between all star-pairs
% dot_products = star_vectors * star_vectors';
% angles_degrees = acosd(dot_products);
% % Tabulate results
% disp('Tabulated results of star angles:')
% fprintf('            Star i                          Star j              Dot Product   Angle (deg)\n')
% for i = 1:length(star_vectors)
%   for j = 1:length(star_vectors)
%     fprintf('(%s)   (%s)      %.4f      %.4f\n', ...
%       sprintf(' %7.4f ', star_vectors(i, :)), ...
%       sprintf(' %7.4f ', star_vectors(j, :)), ...
%       dot_products(i, j), ...
%       angles_degrees(i, j))
%   end
% end

% save('A2P3_computed.mat', 'cleared_image', 'stars', 'star_vectors', 'dot_products', 'angles_degrees');
