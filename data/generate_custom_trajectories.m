% GENERATE_CUSTOM_TRAJECTORIES
% Generates 2D PLANAR trajectories for myPalletizer 260 Pi:
% 1. "THAY HUY DEP TRAI" (Width = 215.5 mm, Height = 120 mm)
% 2. "LOGO BACH KHOA HCM" (Width = 138.6 mm, Height = 140 mm)
%
% Strictly 2D Planar on the vertical plane X = 245.0 mm.
% Optimized to stay strictly on the geometry contour (no stray moves outside).

clear; clc;
addpath('d:\Lidar\myPalletizer_BTL\kinematics');
addpath('d:\Lidar\myPalletizer_BTL\gui');

% 2D Planar Canvas Configuration (mm)
X_plane = 245.0; % 2D drawing board plane (constant X for all DRAW actions)
X_lift  = 237.0; % Retracted pen plane for transit (MOVE actions)
phi_val = 0.0;   % Wrist horizontal (deg)
t_draw  = 0.40;  % Duration per drawing stroke (s)
t_lift  = 0.25;  % Duration per lift/transit (s)

%% ========================================================================
%% PART 1: SUPER-SIZED "THẦY HUY ĐẸP TRAI" (Strictly on-path 2D)
%% ========================================================================
fprintf('Generating Super-Sized Trajectory 1: "THAY HUY DEP TRAI"...\n');

% Letter stroke definitions in normalized [0..1] x [0..1]:
strokes_T = { [0, 1; 1, 1], [0.5, 1; 0.5, 0] };
strokes_H = { [0, 0; 0, 1], [1, 0; 1, 1], [0, 0.5; 1, 0.5] };
strokes_A = { [0, 0; 0.5, 1; 1, 0], [0.2, 0.35; 0.8, 0.35] };
strokes_A_hat = { [0.25, 1.08; 0.5, 1.25; 0.75, 1.08] }; % Hat ^
strokes_A_grave = { [0.65, 1.35; 0.40, 1.20] };           % Grave ` (dấu huyền)
strokes_Y = { [0, 1; 0.5, 0.5], [1, 1; 0.5, 0.5; 0.5, 0] };
strokes_U = { [0, 1; 0, 0.15; 0.3, 0; 0.7, 0; 1, 0.15; 1, 1] };

strokes_D = { [0, 0; 0, 1; 0.6, 1; 1, 0.75; 1, 0.25; 0.6, 0; 0, 0] };
strokes_D_bar = { [-0.2, 0.65; 0.5, 0.65] }; % Gạch chữ Đ
strokes_E = { [1, 1; 0, 1; 0, 0; 1, 0], [0, 0.5; 0.75, 0.5] };
strokes_E_hat = { [0.25, 1.08; 0.5, 1.25; 0.75, 1.08] }; % Mũ chữ Ê
strokes_E_dot = { [0.45, -0.15; 0.55, -0.15] };           % Dấu nặng .
strokes_P = { [0, 0; 0, 1; 0.8, 1; 1, 0.8; 1, 0.55; 0.7, 0.45; 0, 0.45] };
strokes_R = { [0, 0; 0, 1; 0.8, 1; 1, 0.8; 1, 0.55; 0.7, 0.45; 0, 0.45], [0.4, 0.45; 1, 0] };
strokes_I = { [0.1, 1; 0.9, 1], [0.5, 1; 0.5, 0], [0.1, 0; 0.9, 0] };

strokes_A_diacritic = [strokes_A, strokes_A_hat, strokes_A_grave];
strokes_D_diacritic = [strokes_D, strokes_D_bar];
strokes_E_diacritic = [strokes_E, strokes_E_hat, strokes_E_dot];

% Expanded Dimensions (mm):
w_char   = 23.5; % Character width
h_char   = 36.0; % Character height
gap_char = 7.0;  % Gap between letters
gap_word = 16.0; % Gap between words

% Line 1: "THẦY HUY"
line1_letters = {strokes_T, strokes_H, strokes_A_diacritic, strokes_Y, ...
                 'SPACE', ...
                 strokes_H, strokes_U, strokes_Y};

% Line 2: "ĐẸP TRAI"
line2_letters = {strokes_D_diacritic, strokes_E_diacritic, strokes_P, ...
                 'SPACE', ...
                 strokes_T, strokes_R, strokes_A, strokes_I};

z_line1 = 150.0; % Baseline of Line 1 (mm) -> reaches up to 198 mm
z_line2 = 85.0;  % Baseline of Line 2 (mm) -> reaches from 78 to 125 mm

w1_total = (7 * w_char) + (5 * gap_char) + gap_word;
y1_start = -w1_total / 2;

w2_total = (7 * w_char) + (5 * gap_char) + gap_word;
y2_start = -w2_total / 2;

fprintf('  Line 1 span: Y in [%.1f, %.1f] mm (width = %.1f mm)\n', y1_start, y1_start + w1_total, w1_total);
fprintf('  Line 2 span: Y in [%.1f, %.1f] mm (width = %.1f mm)\n', y2_start, y2_start + w2_total, w2_total);

wp_list1 = [];
wp_id = 1;

% Flatten all strokes of Line 1 and Line 2 into an ordered cell array of strokes
all_text_strokes = {};

curr_y = y1_start;
for i = 1:numel(line1_letters)
    item = line1_letters{i};
    if ischar(item) && strcmp(item, 'SPACE')
        curr_y = curr_y + gap_word;
        continue;
    end
    for s = 1:numel(item)
        stroke = item{s};
        pts_y = curr_y + stroke(:, 1) * w_char;
        pts_z = z_line1 + stroke(:, 2) * h_char;
        all_text_strokes{end+1} = [pts_y, pts_z]; %#ok<AGROW>
    end
    curr_y = curr_y + w_char + gap_char;
end

curr_y = y2_start;
for i = 1:numel(line2_letters)
    item = line2_letters{i};
    if ischar(item) && strcmp(item, 'SPACE')
        curr_y = curr_y + gap_word;
        continue;
    end
    for s = 1:numel(item)
        stroke = item{s};
        pts_y = curr_y + stroke(:, 1) * w_char;
        pts_z = z_line2 + stroke(:, 2) * h_char;
        all_text_strokes{end+1} = [pts_y, pts_z]; %#ok<AGROW>
    end
    curr_y = curr_y + w_char + gap_char;
end

% Build waypoint list: Starts directly at stroke 1, no outer Home position
for s = 1:numel(all_text_strokes)
    st = all_text_strokes{s};
    if s == 1
        % Direct start at first point
        wp_list1 = [wp_list1; wp_id, X_plane, st(1, 1), st(1, 2), phi_val, 1.0, 1];
        wp_id = wp_id + 1;
    else
        % Lift pen at previous end, move directly to current start, lower to 2D plane
        prev_end = all_text_strokes{s-1}(end, :);
        % Lift
        wp_list1 = [wp_list1; wp_id, X_lift, prev_end(1), prev_end(2), phi_val, 0.15, 0];
        wp_id = wp_id + 1;
        % Transit in air
        wp_list1 = [wp_list1; wp_id, X_lift, st(1, 1), st(1, 2), phi_val, t_lift, 0];
        wp_id = wp_id + 1;
        % Lower to draw
        wp_list1 = [wp_list1; wp_id, X_plane, st(1, 1), st(1, 2), phi_val, 0.15, 1];
        wp_id = wp_id + 1;
    end
    
    % Draw stroke points
    for p = 2:size(st, 1)
        dist_p = hypot(st(p, 1) - st(p-1, 1), st(p, 2) - st(p-1, 2));
        dur_p = max(0.20, min(0.60, dist_p / 45.0));
        wp_list1 = [wp_list1; wp_id, X_plane, st(p, 1), st(p, 2), phi_val, dur_p, 1];
        wp_id = wp_id + 1;
    end
end

% Reachability validation
fprintf('Validating %d points for "THAY HUY DEP TRAI"...\n', size(wp_list1, 1));
for k = 1:size(wp_list1, 1)
    [th, succ] = inverse_kinematics(wp_list1(k, 2), wp_list1(k, 3), wp_list1(k, 4), deg2rad(wp_list1(k, 5)));
    if ~succ
        error('Waypoint %d [%.1f, %.1f, %.1f] is unreachable!', k, wp_list1(k, 2), wp_list1(k, 3), wp_list1(k, 4));
    end
    if ~check_limits(th, 'joints')
        error('Waypoint %d violates joint limits!', k);
    end
end
fprintf('All %d waypoints are 100%% REACH-VALIDATED!\n', size(wp_list1, 1));

% Write CSV & JSON
csv_file1 = 'd:\Lidar\myPalletizer_BTL\data\trajectory_thay_huy_dep_trai.csv';
fid = fopen(csv_file1, 'w');
fprintf(fid, 'point_id,x,y,z,phi,duration,action\n');
for k = 1:size(wp_list1, 1)
    if wp_list1(k, 7) == 1
        act = 'DRAW';
    else
        act = 'MOVE';
    end
    fprintf(fid, '%d,%.1f,%.1f,%.1f,%.1f,%.2f,%s\n', ...
        wp_list1(k, 1), wp_list1(k, 2), wp_list1(k, 3), wp_list1(k, 4), wp_list1(k, 5), wp_list1(k, 6), act);
end
fclose(fid);
fprintf('Saved CSV: %s\n', csv_file1);

json_file1 = 'd:\Lidar\myPalletizer_BTL\data\trajectory_thay_huy_dep_trai.json';
write_json_traj(json_file1, 'Vietnamese Calligraphy: THAY HUY DEP TRAI (2D Planar)', ...
    'Robot writes "THAY HUY DEP TRAI" in strictly 2D planar canvas X=245mm', wp_list1);
fprintf('Saved JSON: %s\n', json_file1);


%% ========================================================================
%% PART 2: EXPANDED CONTINUOUS HCMUT CREST ("BIỂU TRƯNG BÁCH KHOA TP.HCM")
%% ========================================================================
fprintf('\nGenerating Official Trajectory 2: "HCMUT CREST"...\n');

% Expanded dimensions:
y_0   = 0.0;   % Center Y (mm)
z_0   = 128.0; % Center Z (mm)
R_hex = 40.0;  % Hexagon radius (mm) -> Width = 138.6 mm, Height = 140 mm!

s3 = sqrt(3);

% Vertices
H1 = [0.0, 1.0];             % Top apex of hexagon
H2 = [-s3/2, 0.5];           % Top-left of hexagon
H3 = [-s3/2, -0.5];          % Bottom-left of hexagon
H4 = [0.0, -1.0];            % Bottom apex of hexagon
H5 = [s3/2, -0.5];           % Bottom-right of hexagon
H6 = [s3/2, 0.5];            % Top-right of hexagon

T3 = [0.0, 2.0];             % Top apex of top cube
T4 = [s3/2, 1.5];
T2 = [-s3/2, 1.5];

R2 = [s3, 0.0];              % Outer right vertices of bottom-right cube
R3 = [s3, -1.0];
R4 = [s3/2, -1.5];

L2 = [-s3/2, -1.5];          % Outer left vertices of bottom-left cube
L3 = [-s3, -1.0];
L4 = [-s3, 0.0];

% Continuous single connected path for entire outer geometry + seams + central hexagon:
% 1. Traces 12 outer edges of the 3 cubes: T3 -> T4 -> H6 -> R2 -> R3 -> R4 -> H4 -> L2 -> L3 -> L4 -> H2 -> T2 -> T3
% 2. Cuts down top seam: T3 -> H1
% 3. Traces hexagon with branching seams: H1 -> H6 -> H5 -> R3 -> H5 -> H4 -> H3 -> L3 -> H3 -> H2 -> H1
geom_path_norm = [ ...
    T3; T4; H6; R2; R3; R4; H4; L2; L3; L4; H2; T2; T3; ... % Outer boundary (12 edges)
    H1; ...                                                 % Top vertical seam
    H6; H5; ...                                             % Hex top-right to bottom-right
    R3; H5; ...                                             % Bottom-right seam and back
    H4; H3; ...                                             % Hex bottom to bottom-left
    L3; H3; ...                                             % Bottom-left seam and back
    H2; H1 ...                                              % Hex left to top apex
];

% --- Central Typography: Bold 'BK' (Upper Half of Hexagon) ---
b_poly = [-0.558, -0.156; ...
          -0.558,  0.470; ...
          -0.200,  0.470; ...
          -0.080,  0.310; ...
          -0.200,  0.150; ...
          -0.558,  0.150; ...
          -0.200,  0.150; ...
          -0.080, -0.010; ...
          -0.200, -0.156; ...
          -0.558, -0.156];

k_spine = [0.150, -0.156; 0.150, 0.470];
k_up    = [0.150,  0.150; 0.650, 0.470];
k_down  = [0.220,  0.190; 0.650, -0.156];

% --- Central Typography: 'TP.HCM' (Lower Half of Hexagon) ---
t_bar  = [-0.600, -0.238; -0.420, -0.238];
t_stem = [-0.510, -0.238; -0.510, -0.470];

p_poly = [-0.370, -0.470; ...
          -0.370, -0.238; ...
          -0.240, -0.238; ...
          -0.240, -0.354; ...
          -0.370, -0.354];

dot_poly = [-0.180, -0.460; -0.170, -0.460];

h_left  = [-0.070, -0.470; -0.070, -0.238];
h_bar   = [-0.070, -0.354;  0.070, -0.354];
h_right = [ 0.070, -0.238;  0.070, -0.470];

c_poly = [0.290, -0.238; 0.180, -0.238; 0.180, -0.470; 0.290, -0.470];

m_poly = [0.380, -0.470; 0.380, -0.238; 0.490, -0.370; 0.600, -0.238; 0.600, -0.470];

inner_text_strokes = {b_poly, k_spine, k_up, k_down, ...
                      t_bar, t_stem, p_poly, dot_poly, h_left, h_bar, h_right, c_poly, m_poly};

wp_list2 = [];
wp_id2 = 1;

% 1. Draw continuous main geometric crest:
pts_geom = [y_0 + geom_path_norm(:, 1) * R_hex, z_0 + geom_path_norm(:, 2) * R_hex];

% Start directly at T3 (top apex)
wp_list2 = [wp_list2; wp_id2, X_plane, pts_geom(1, 1), pts_geom(1, 2), phi_val, 1.0, 1];
wp_id2 = wp_id2 + 1;

for p = 2:size(pts_geom, 1)
    dist_p = hypot(pts_geom(p, 1) - pts_geom(p-1, 1), pts_geom(p, 2) - pts_geom(p-1, 2));
    dur_p = max(0.20, min(0.60, dist_p / 40.0));
    wp_list2 = [wp_list2; wp_id2, X_plane, pts_geom(p, 1), pts_geom(p, 2), phi_val, dur_p, 1];
    wp_id2 = wp_id2 + 1;
end

% 2. Draw typography inside hexagon:
last_pt = pts_geom(end, :); % H1

for s = 1:numel(inner_text_strokes)
    st = inner_text_strokes{s};
    st_pts = [y_0 + st(:, 1) * R_hex, z_0 + st(:, 2) * R_hex];
    
    % Lift pen at last_pt, step to st_pts(1,:), lower to draw
    wp_list2 = [wp_list2; wp_id2, X_lift, last_pt(1), last_pt(2), phi_val, 0.15, 0];
    wp_id2 = wp_id2 + 1;
    
    wp_list2 = [wp_list2; wp_id2, X_lift, st_pts(1, 1), st_pts(1, 2), phi_val, t_lift, 0];
    wp_id2 = wp_id2 + 1;
    
    wp_list2 = [wp_list2; wp_id2, X_plane, st_pts(1, 1), st_pts(1, 2), phi_val, 0.15, 1];
    wp_id2 = wp_id2 + 1;
    
    for p = 2:size(st_pts, 1)
        dist_p = hypot(st_pts(p, 1) - st_pts(p-1, 1), st_pts(p, 2) - st_pts(p-1, 2));
        dur_p = max(0.15, min(0.50, dist_p / 35.0));
        wp_list2 = [wp_list2; wp_id2, X_plane, st_pts(p, 1), st_pts(p, 2), phi_val, dur_p, 1];
        wp_id2 = wp_id2 + 1;
    end
    
    last_pt = st_pts(end, :);
end

% Stop right at last point of 'M' (NO returning to Home)
fprintf('Validating %d points for HCMUT Crest...\n', size(wp_list2, 1));
for k = 1:size(wp_list2, 1)
    [th, succ] = inverse_kinematics(wp_list2(k, 2), wp_list2(k, 3), wp_list2(k, 4), deg2rad(wp_list2(k, 5)));
    if ~succ
        error('Waypoint %d [%.1f, %.1f, %.1f] is unreachable!', k, wp_list2(k, 2), wp_list2(k, 3), wp_list2(k, 4));
    end
    if ~check_limits(th, 'joints')
        error('Waypoint %d violates joint limits!', k);
    end
end
fprintf('All %d waypoints are 100%% REACH-VALIDATED and within joint limits!\n', size(wp_list2, 1));

csv_file2 = 'd:\Lidar\myPalletizer_BTL\data\trajectory_logo_bach_khoa.csv';
fid = fopen(csv_file2, 'w');
fprintf(fid, 'point_id,x,y,z,phi,duration,action\n');
for k = 1:size(wp_list2, 1)
    if wp_list2(k, 7) == 1
        act = 'DRAW';
    else
        act = 'MOVE';
    end
    fprintf(fid, '%d,%.1f,%.1f,%.1f,%.1f,%.2f,%s\n', ...
        wp_list2(k, 1), wp_list2(k, 2), wp_list2(k, 3), wp_list2(k, 4), wp_list2(k, 5), wp_list2(k, 6), act);
end
fclose(fid);
fprintf('Saved HCMUT Crest CSV: %s\n', csv_file2);

json_file2 = 'd:\Lidar\myPalletizer_BTL\data\trajectory_logo_bach_khoa.json';
write_json_traj(json_file2, 'Emblem Drawing: HCMUT Crest (Official 2D Planar)', ...
    'Robot draws official HCMUT crest with central regular hexagon, 3 isometric cubes, BK and TP.HCM in 2D plane X=245mm', wp_list2);
fprintf('Saved HCMUT Crest JSON: %s\n', json_file2);

fprintf('\nSUCCESS: Both trajectories generated and verified!\n');

function write_json_traj(filepath, name, desc, wp_matrix)
    data = struct();
    data.trajectory_name = name;
    data.version = '1.0';
    data.robot = 'myPalletizer 260 Pi';
    data.description = desc;
    data.units = struct('length', 'mm', 'angle', 'deg', 'time', 's');
    
    wps = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, 'phi', {}, 'duration', {}, 'action', {});
    for i = 1:size(wp_matrix, 1)
        if wp_matrix(i, 7) == 1
            act = 'DRAW';
        else
            act = 'MOVE';
        end
        wps(i).id = wp_matrix(i, 1);
        wps(i).x = wp_matrix(i, 2);
        wps(i).y = wp_matrix(i, 3);
        wps(i).z = wp_matrix(i, 4);
        wps(i).phi = wp_matrix(i, 5);
        wps(i).duration = wp_matrix(i, 6);
        wps(i).action = act;
    end
    data.waypoints = wps;
    
    txt = jsonencode(data, 'PrettyPrint', true);
    fid = fopen(filepath, 'w');
    fwrite(fid, txt, 'char');
    fclose(fid);
end
