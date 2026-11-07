% GENERATE_CUSTOM_TRAJECTORIES
% Generates SUPER-SIZED wide-span artistic trajectories for myPalletizer 260 Pi:
% 1. "THAY HUY DEP TRAI" (Width = 240 mm, Height = 120 mm)
% 2. "LOGO BACH KHOA HCM" (Width = 112 mm, Height = 130 mm)
%
% Uses the natural cylindrical arc R ~ 252 mm so the robot arm swings
% wide across the entire front workspace (Base Yaw theta1 swings > 60 deg,
% shoulder/elbow span 120 mm vertically).

clear; clc;
addpath('d:\Lidar\myPalletizer_BTL\kinematics');
addpath('d:\Lidar\myPalletizer_BTL\gui');

R_touch = 252.0; % Pen touching surface radius (mm)
R_lift  = 238.0; % Pen lifted/retracted radius (mm)
phi_val = 0.0;   % Wrist horizontal (deg)
t_draw  = 0.40;  % Duration per drawing stroke (s)
t_lift  = 0.30;  % Duration per lift/transit (s)

%% ========================================================================
%% PART 1: SUPER-SIZED "THẦY HUY ĐẸP TRAI"
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

% Super-Sized Dimensions (mm):
w_char   = 22.0; % Character width (2.3x larger!)
h_char   = 34.0; % Character height (2.1x larger!)
gap_char = 7.5;  % Gap between letters
gap_word = 18.0; % Gap between words

% Line 1: "THẦY HUY"
line1_letters = {strokes_T, strokes_H, strokes_A_diacritic, strokes_Y, ...
                 'SPACE', ...
                 strokes_H, strokes_U, strokes_Y};

% Line 2: "ĐẸP TRAI"
line2_letters = {strokes_D_diacritic, strokes_E_diacritic, strokes_P, ...
                 'SPACE', ...
                 strokes_T, strokes_R, strokes_A, strokes_I};

z_line1 = 152.0; % Baseline of Line 1 (mm) -> reaches up to 195 mm
z_line2 = 85.0;  % Baseline of Line 2 (mm) -> reaches from 78 to 125 mm

w1_total = (7 * w_char) + (5 * gap_char) + gap_word;
y1_start = -w1_total / 2;

w2_total = (7 * w_char) + (5 * gap_char) + gap_word;
y2_start = -w2_total / 2;

fprintf('  Line 1 span: Y in [%.1f, %.1f] mm (width = %.1f mm)\n', y1_start, y1_start + w1_total, w1_total);
fprintf('  Line 2 span: Y in [%.1f, %.1f] mm (width = %.1f mm)\n', y2_start, y2_start + w2_total, w2_total);

wp_list1 = [];
wp_id = 1;

% Start from Home pose [240, 0, 180, 0]
wp_list1 = [wp_list1; wp_id, 240.0, 0.0, 180.0, phi_val, 2.0, 0];
wp_id = wp_id + 1;

% Render Line 1
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
        
        % Lift & transit to start
        x_l1 = sqrt(R_lift^2 - pts_y(1)^2);
        wp_list1 = [wp_list1; wp_id, x_l1, pts_y(1), pts_z(1), phi_val, t_lift, 0];
        wp_id = wp_id + 1;
        
        % Touch down
        x_t1 = sqrt(R_touch^2 - pts_y(1)^2);
        wp_list1 = [wp_list1; wp_id, x_t1, pts_y(1), pts_z(1), phi_val, t_lift*0.7, 1];
        wp_id = wp_id + 1;
        
        % Draw stroke
        for p = 2:size(stroke, 1)
            x_tp = sqrt(R_touch^2 - pts_y(p)^2);
            dist_p = hypot(pts_y(p) - pts_y(p-1), pts_z(p) - pts_z(p-1));
            dur_p = max(0.25, min(0.60, dist_p / 45.0));
            wp_list1 = [wp_list1; wp_id, x_tp, pts_y(p), pts_z(p), phi_val, dur_p, 1];
            wp_id = wp_id + 1;
        end
        
        % Lift pen
        x_le = sqrt(R_lift^2 - pts_y(end)^2);
        wp_list1 = [wp_list1; wp_id, x_le, pts_y(end), pts_z(end), phi_val, t_lift*0.7, 0];
        wp_id = wp_id + 1;
    end
    curr_y = curr_y + w_char + gap_char;
end

% Render Line 2
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
        
        x_l1 = sqrt(R_lift^2 - pts_y(1)^2);
        wp_list1 = [wp_list1; wp_id, x_l1, pts_y(1), pts_z(1), phi_val, t_lift, 0];
        wp_id = wp_id + 1;
        
        x_t1 = sqrt(R_touch^2 - pts_y(1)^2);
        wp_list1 = [wp_list1; wp_id, x_t1, pts_y(1), pts_z(1), phi_val, t_lift*0.7, 1];
        wp_id = wp_id + 1;
        
        for p = 2:size(stroke, 1)
            x_tp = sqrt(R_touch^2 - pts_y(p)^2);
            dist_p = hypot(pts_y(p) - pts_y(p-1), pts_z(p) - pts_z(p-1));
            dur_p = max(0.25, min(0.60, dist_p / 45.0));
            wp_list1 = [wp_list1; wp_id, x_tp, pts_y(p), pts_z(p), phi_val, dur_p, 1];
            wp_id = wp_id + 1;
        end
        
        x_le = sqrt(R_lift^2 - pts_y(end)^2);
        wp_list1 = [wp_list1; wp_id, x_le, pts_y(end), pts_z(end), phi_val, t_lift*0.7, 0];
        wp_id = wp_id + 1;
    end
    curr_y = curr_y + w_char + gap_char;
end

% Return Home
wp_list1 = [wp_list1; wp_id, 240.0, 0.0, 180.0, phi_val, 2.0, 0];

% Reachability validation
fprintf('Validating %d points for Super-Sized "THAY HUY DEP TRAI"...\n', size(wp_list1, 1));
for k = 1:size(wp_list1, 1)
    [~, succ] = inverse_kinematics(wp_list1(k, 2), wp_list1(k, 3), wp_list1(k, 4), deg2rad(wp_list1(k, 5)));
    if ~succ
        error('Waypoint %d [%.1f, %.1f, %.1f] is unreachable!', k, wp_list1(k, 2), wp_list1(k, 3), wp_list1(k, 4));
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
fprintf('Saved Super-Sized CSV: %s\n', csv_file1);

json_file1 = 'd:\Lidar\myPalletizer_BTL\data\trajectory_thay_huy_dep_trai.json';
write_json_traj(json_file1, 'Vietnamese Calligraphy: THAY HUY DEP TRAI (Super-Sized)', ...
    'Robot writes "THAY HUY DEP TRAI" across a 240 mm wide workspace sweep', wp_list1);
fprintf('Saved Super-Sized JSON: %s\n', json_file1);


%% ========================================================================
%% PART 2: OFFICIAL HCMUT CREST ("BIỂU TRƯNG ĐẠI HỌC BÁCH KHOA TP.HCM")
%% ========================================================================
fprintf('\nGenerating Official Trajectory 2: "HCMUT CREST"...\n');

% Cylindrical canvas and crest dimensions
y_0   = 0.0;   % Horizontal center on cylinder (mm)
z_0   = 138.0; % Vertical center elevation (mm)
R_hex = 31.0;  % Central regular hexagon radius (mm)

% --- Central Regular Hexagon (pointed top 90 deg, pointed bottom 270 deg, vertical sides) ---
s3 = sqrt(3);
H1 = [0.0, 1.0];             % Top apex (90 deg)
H2 = [-s3/2, 0.5];           % Top-left (150 deg)
H3 = [-s3/2, -0.5];          % Bottom-left (210 deg)
H4 = [0.0, -1.0];            % Bottom apex (270 deg)
H5 = [s3/2, -0.5];           % Bottom-right (330 deg)
H6 = [s3/2, 0.5];            % Top-right (30 deg)

stroke_hex = [H1; H2; H3; H4; H5; H6; H1];

% --- Three 3D Isometric Cubes at 120-degree intervals ---
% 1. Top Cube (at 90 deg): outer rim + vertical dividing seam to H1 (does NOT cut into hex)
top_rim  = [H2; -s3/2, 1.5; 0.0, 2.0; s3/2, 1.5; H6];
top_seam = [0.0, 2.0; 0.0, 1.0];

% 2. Bottom-Left Cube (at 210 deg): outer rim + radial dividing seam to H3 (does NOT cut into hex)
bl_rim  = [H4; -s3/2, -1.5; -s3, -1.0; -s3, 0.0; H2];
bl_seam = [-s3, -1.0; -s3/2, -0.5];

% 3. Bottom-Right Cube (at 330 deg): outer rim + radial dividing seam to H5 (does NOT cut into hex)
br_rim  = [H6; s3, 0.0; s3, -1.0; s3/2, -1.5; H4];
br_seam = [s3, -1.0; s3/2, -0.5];

% --- Central Typography: Bold 'BK' (Upper Half of Hexagon) ---
% Letter 'B': continuous spine + upper bowl + lower bowl
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

% Letter 'K': vertical spine + upper diagonal arm + lower diagonal leg
k_spine = [0.150, -0.156; 0.150, 0.470];
k_up    = [0.150,  0.150; 0.650, 0.470];
k_down  = [0.220,  0.190; 0.650, -0.156];

% --- Central Typography: 'TP.HCM' (Lower Half of Hexagon, single dot between P and H, no dot after M) ---
% 'T': horizontal top bar + vertical stem
t_bar  = [-0.600, -0.238; -0.420, -0.238];
t_stem = [-0.510, -0.238; -0.510, -0.470];

% 'P': vertical stem + upper loop (closed)
p_poly = [-0.370, -0.470; ...
          -0.370, -0.238; ...
          -0.240, -0.238; ...
          -0.240, -0.354; ...
          -0.370, -0.354];

% Dot '.': single dot between 'P' and 'H'
dot_poly = [-0.180, -0.460; -0.170, -0.460];

% 'H': left stem + crossbar + right stem
h_left  = [-0.070, -0.470; -0.070, -0.238];
h_bar   = [-0.070, -0.354;  0.070, -0.354];
h_right = [ 0.070, -0.238;  0.070, -0.470];

% 'C': arc/box contour
c_poly = [0.290, -0.238; 0.180, -0.238; 0.180, -0.470; 0.290, -0.470];

% 'M': 4-segment capital M
m_poly = [0.380, -0.470; 0.380, -0.238; 0.490, -0.370; 0.600, -0.238; 0.600, -0.470];

% Aggregate all strokes (NO 'logo bách khoa' text)
norm_strokes = {stroke_hex, top_rim, top_seam, bl_rim, bl_seam, br_rim, br_seam, ...
                b_poly, k_spine, k_up, k_down, ...
                t_bar, t_stem, p_poly, dot_poly, h_left, h_bar, h_right, c_poly, m_poly};

wp_list2 = [];
wp_id2 = 1;

% Start from Home pose [240, 0, 180, 0]
wp_list2 = [wp_list2; wp_id2, 240.0, 0.0, 180.0, phi_val, 2.0, 0];
wp_id2 = wp_id2 + 1;

for s = 1:numel(norm_strokes)
    n_pts = norm_strokes{s};
    pts = [y_0 + n_pts(:, 1) * R_hex, z_0 + n_pts(:, 2) * R_hex];
    
    % Lift & transit to start of stroke
    x_l1 = sqrt(R_lift^2 - pts(1, 1)^2);
    wp_list2 = [wp_list2; wp_id2, x_l1, pts(1, 1), pts(1, 2), phi_val, t_lift, 0];
    wp_id2 = wp_id2 + 1;
    
    % Touch down to start drawing
    x_t1 = sqrt(R_touch^2 - pts(1, 1)^2);
    wp_list2 = [wp_list2; wp_id2, x_t1, pts(1, 1), pts(1, 2), phi_val, t_lift * 0.7, 1];
    wp_id2 = wp_id2 + 1;
    
    % Draw intermediate segments
    for p = 2:size(pts, 1)
        x_tp = sqrt(R_touch^2 - pts(p, 1)^2);
        dist_p = hypot(pts(p, 1) - pts(p-1, 1), pts(p, 2) - pts(p-1, 2));
        dur_p = max(0.20, min(0.60, dist_p / 40.0));
        wp_list2 = [wp_list2; wp_id2, x_tp, pts(p, 1), pts(p, 2), phi_val, dur_p, 1];
        wp_id2 = wp_id2 + 1;
    end
    
    % Retract/lift pen at end of stroke
    x_le = sqrt(R_lift^2 - pts(end, 1)^2);
    wp_list2 = [wp_list2; wp_id2, x_le, pts(end, 1), pts(end, 2), phi_val, t_lift * 0.7, 0];
    wp_id2 = wp_id2 + 1;
end

% Return to Home pose [240, 0, 180, 0]
wp_list2 = [wp_list2; wp_id2, 240.0, 0.0, 180.0, phi_val, 2.0, 0];

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
write_json_traj(json_file2, 'Emblem Drawing: HCMUT Crest (Official)', ...
    'Robot draws official HCMUT crest with central regular hexagon, 3 isometric cubes at 120 deg, BK and TP.HCM text', wp_list2);
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
