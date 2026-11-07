function [grid_points, is_valid, details] = pallet_grid_generator(origin, rows, cols, layers, spacing)
% PALLET_GRID_GENERATOR Calculates drop coordinates for an M x N x K pallet grid.
%
% Computes structured 3D spatial target locations for palletizing stacking:
%   P(i, j, k) = P_origin + (i - 1)*dx + (j - 1)*dy + (k - 1)*dz
% and validates workspace compliance against myPalletizer 260 Pi limits.
%
% Syntax:
%   grid_points = pallet_grid_generator(origin, rows, cols, layers, spacing)
%   [grid_points, is_valid, details] = pallet_grid_generator(origin, rows, cols, layers, spacing)
%
% Inputs:
%   origin  - [1x3] or [3x1] array [x0, y0, z0] pallet origin reference (mm)
%   rows    - Number of rows along X-axis (M, positive integer)
%   cols    - Number of columns along Y-axis (N, positive integer)
%   layers  - Number of stacking layers along Z-axis (K, positive integer)
%   spacing - [1x3] or [3x1] array [dx, dy, dz] cell spacings (mm, strictly positive)
%
% Outputs:
%   grid_points - [(M*N*K) x 3] matrix of 3D drop coordinates [x, y, z] in mm
%   is_valid    - Logical flag: true if all grid points lie within robot reach
%                 envelope (R <= 260 mm, -15 mm <= z <= 358 mm)
%   details     - Struct containing diagnostic metrics:
%                   .total_points - Total number of grid positions (M*N*K)
%                   .radii        - [(M*N*K) x 1] radial distances from base
%                   .is_in_reach  - [(M*N*K) x 1] logical flags per point
%                   .max_radius   - Maximum radial distance (mm)
%                   .z_min, z_max - Minimum and maximum Z elevations (mm)
%                   .message      - Diagnostic message
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Input validation
    if nargin < 5
        error('pallet_grid_generator:TooFewInputs', ...
              'Must provide origin, rows, cols, layers, and spacing.');
    end

    origin = reshape(origin, 1, 3);
    spacing = reshape(spacing, 1, 3);

    if any(spacing <= 0)
        error('pallet_grid_generator:InvalidSpacing', ...
              'All spacing components [dx, dy, dz] must be strictly positive.');
    end

    if rows < 1 || cols < 1 || layers < 1 || ...
       rows ~= round(rows) || cols ~= round(cols) || layers ~= round(layers)
        error('pallet_grid_generator:InvalidDimensions', ...
              'Grid dimensions (rows, cols, layers) must be positive integers.');
    end

    x0 = origin(1);
    y0 = origin(2);
    z0 = origin(3);

    dx = spacing(1);
    dy = spacing(2);
    dz = spacing(3);

    total_points = rows * cols * layers;
    grid_points = zeros(total_points, 3);
    idx = 1;

    % Loop ordering: Layers (Z), then Rows (X), then Columns (Y)
    for k = 1:layers
        for i = 1:rows
            for j = 1:cols
                gx = x0 + (i - 1) * dx;
                gy = y0 + (j - 1) * dy;
                gz = z0 + (k - 1) * dz;
                grid_points(idx, :) = [gx, gy, gz];
                idx = idx + 1;
            end
        end
    end

    % Reachability & envelope validation
    % myPalletizer 260 Pi nominal operational envelope:
    R_MAX = 260.0; % mm working radius
    Z_MIN = -15.0; % mm lower bound
    Z_MAX = 358.0; % mm upper bound

    radii = sqrt(grid_points(:, 1).^2 + grid_points(:, 2).^2);
    z_vals = grid_points(:, 3);

    is_in_reach = (radii <= R_MAX + 1e-6) & (z_vals >= Z_MIN - 1e-6) & (z_vals <= Z_MAX + 1e-6);
    is_valid = all(is_in_reach);

    details = struct();
    details.total_points = total_points;
    details.rows = rows;
    details.cols = cols;
    details.layers = layers;
    details.radii = radii;
    details.is_in_reach = is_in_reach;
    details.max_radius = max(radii);
    details.z_min = min(z_vals);
    details.z_max = max(z_vals);

    if is_valid
        details.message = sprintf('All %d pallet grid coordinates lie within workspace envelope (max R=%.1f mm).', ...
                                  total_points, details.max_radius);
    else
        out_count = sum(~is_in_reach);
        details.message = sprintf('%d of %d pallet grid coordinates exceed workspace envelope (max R=%.1f mm, R_max=%.1f mm).', ...
                                  out_count, total_points, details.max_radius, R_MAX);
    end
end
