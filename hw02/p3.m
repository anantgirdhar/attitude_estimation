fprintf('Problem 3\n\n\n')
% Author: Anant Girdhar
% Date: 2024-09-24

load A2P3.mat;
figure;
image(p3_image);
colormap(parula(max(max(p3_image))));
colorbar;
xlabel('u');
ylabel('v');

% Part (a)
% Remove noisy measurements
cleared_image = p3_image;
cleared_image(p3_image < 30) = 0;

% Create heat map
figure;
image(cleared_image);
colormap(parula(max(max(p3_image))));
colorbar;
xlabel('u');
ylabel('v');


% Part (b)
disp('Part (b)')
% Estimate star positions
% Reminder: the image matrix is indexed (v, u)
% Create a list of coordinates for the identified stars
stars = [];
image_size = size(cleared_image);
% Create a list of coordinates with nonzero values that we've already processed
processed_cells = [];
for u = 1 : image_size(2)
  for v = 1 : image_size(1)
    if cleared_image(v, u) > 0
      if length(processed_cells) > 0 && ismember([u v], processed_cells, 'rows')
        continue
      end
      % This cell is part of a star
      % Now we need to find all illuminated cells that are orthogonally connected to this cell
      % Employ a Depth First Search type algorithm to identify which cells belong to this star
      % fprintf('Initiating from (%d %d)\n', u, v)
      [star_coords, photon_count, processed_cells] = locate_star_at(cleared_image, u, v, processed_cells);
      % fprintf('Found star at (%d %d)\n', star_coords)
      stars = [stars ; star_coords];
    end
  end
end
disp('Star coordinates found:')
disp(stars')
fprintf('\n')

% Part (c)
disp('Part (c)')
% Find line of sight vectors for each star
star_vectors = stars;
f = 60;
u_p = 6;
v_p = 6;
% First do an origin shift in the sensor plane relative to (u_p, v_p)
star_vectors(:, 1) = star_vectors(:, 1) - u_p;
star_vectors(:, 2) = star_vectors(:, 2) - v_p;
star_vectors(:, 3) = f;
% Now normalize each row of vectors
star_vectors = star_vectors ./ vecnorm(star_vectors')';
disp('Line of sight unit vectors computed:')
disp(star_vectors')
fprintf('\n')


% Part (d)
disp('Part (d)')
% Compute angles between all star-pairs
dot_products = star_vectors * star_vectors';
angles_degrees = acosd(dot_products);
% Tabulate results
disp('Tabulated results of star angles:')
fprintf('            Star i                          Star j              Dot Product   Angle (deg)\n')
for i = 1:length(star_vectors)
  for j = 1:length(star_vectors)
    fprintf('(%s)   (%s)      %.4f      %.4f\n', ...
      sprintf(' %7.4f ', star_vectors(i, :)), ...
      sprintf(' %7.4f ', star_vectors(j, :)), ...
      dot_products(i, j), ...
      angles_degrees(i, j))
  end
end

save('A2P3_computed.mat', 'cleared_image', 'stars', 'star_vectors', 'dot_products', 'angles_degrees');


function [star_coords, photon_count, processed_cells] = locate_star_at(image, u, v, processed_cells)
  image_size = size(image);
  if v > image_size(1) || ...
      v < 1 || ...
      u > image_size(2) || ...
      u < 1 || ...
      (length(processed_cells) > 0 && ismember([u v], processed_cells, 'rows')) || ...
      image(v, u) == 0
    star_coords = [0 0];
    photon_count = 0;
    % fprintf('Ending at (%d, %d)\n', u, v)
    return
  end
  % Initialize the star coordinates for this star
  % If this cell has already been visited, then we don't need to do anything
  % Find the weighted cell centroid and mark this cell as processed
  % fprintf('At (%d, %d)...\n', u, v)
  photon_count = image(v, u);
  star_coords = [u - 0.5, v - 0.5] * photon_count;
  processed_cells = [processed_cells ; [u, v]];
  % Now we just need to check the cells to the left, right, top, and bottom of this cell
  % Remember that each call to locate_star_at() will return star coordinates divided by the photon_count
  % This division operation needs to be undone before adding the contribution to the star coordinates
  [star_coords_contribution, photon_count_contribution, processed_cells] = locate_star_at(image, u+1, v, processed_cells);
  star_coords = star_coords + star_coords_contribution * photon_count_contribution;
  photon_count = photon_count + photon_count_contribution;
  [star_coords_contribution, photon_count_contribution, processed_cells] = locate_star_at(image, u, v+1, processed_cells);
  star_coords = star_coords + star_coords_contribution * photon_count_contribution;
  photon_count = photon_count + photon_count_contribution;
  [star_coords_contribution, photon_count_contribution, processed_cells] = locate_star_at(image, u-1, v, processed_cells);
  star_coords = star_coords + star_coords_contribution * photon_count_contribution;
  photon_count = photon_count + photon_count_contribution;
  [star_coords_contribution, photon_count_contribution, processed_cells] = locate_star_at(image, u, v-1, processed_cells);
  star_coords = star_coords + star_coords_contribution * photon_count_contribution;
  photon_count = photon_count + photon_count_contribution;
  % Normalize the star coordinates to complete the averaging process
  star_coords = star_coords / photon_count;
end
