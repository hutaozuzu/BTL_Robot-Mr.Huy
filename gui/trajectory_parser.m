function [waypoints, is_valid, error_msg] = trajectory_parser(filepath)
% TRAJECTORY_PARSER Imports and validates trajectory via-points from CSV or JSON files.
%
% Syntax:
%   [waypoints, is_valid, error_msg] = trajectory_parser(filepath)
%
% Inputs:
%   filepath    - Path to .csv or .json trajectory file.
%
% Outputs:
%   waypoints   - Struct array with standardized fields:
%                   .id       (numeric integer ID)
%                   .x        (Cartesian X in mm, double)
%                   .y        (Cartesian Y in mm, double)
%                   .z        (Cartesian Z in mm, double)
%                   .phi      (Pitch angle in degrees, double)
%                   .duration (Segment duration in seconds, double)
%                   .action   (Action tag: 'MOVE', 'PICK', 'PLACE', etc., char)
%   is_valid    - Logical flag: true if parsing and all validation checks pass.
%   error_msg   - Descriptive error string if invalid; empty ('') if valid.
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Default empty struct with exact 7 standard fields
    empty_wp = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, 'phi', {}, 'duration', {}, 'action', {});
    waypoints = empty_wp;
    is_valid = false;
    error_msg = '';

    % 1. Input argument check
    if nargin < 1 || isempty(filepath)
        error_msg = 'Trajectory file path cannot be empty.';
        return;
    end

    if ~ischar(filepath) && ~isstring(filepath)
        error_msg = 'Trajectory file path must be a character vector or string.';
        return;
    end
    filepath = char(filepath);

    % 2. File existence check
    if ~exist(filepath, 'file')
        error_msg = sprintf('Trajectory file does not exist: %s', filepath);
        return;
    end

    % 3. Extension check
    [~, ~, ext] = fileparts(filepath);
    ext = lower(ext);
    if ~ismember(ext, {'.csv', '.json'})
        error_msg = sprintf('Unsupported file extension "%s". Only .csv and .json are supported.', ext);
        return;
    end

    try
        switch ext
            case '.csv'
                [waypoints, is_valid, error_msg] = parse_csv(filepath, empty_wp);
            case '.json'
                [waypoints, is_valid, error_msg] = parse_json(filepath, empty_wp);
        end

        if ~is_valid
            waypoints = empty_wp;
            return;
        end

        % 4. Validate waypoints content and reachability
        [is_valid, error_msg] = validate_waypoints(waypoints);
        if ~is_valid
            waypoints = empty_wp;
        end

    catch ME
        waypoints = empty_wp;
        is_valid = false;
        error_msg = sprintf('Unexpected parser exception: %s', ME.message);
    end
end

%% Helper: Parse CSV
function [waypoints, is_valid, error_msg] = parse_csv(filepath, empty_wp)
    waypoints = empty_wp;
    is_valid = false;
    error_msg = '';

    % Check if file is readable and non-empty
    raw_content = fileread(filepath);
    if isempty(strtrim(raw_content))
        error_msg = 'CSV file is completely empty.';
        return;
    end

    % Read table with preserved variable names
    opts = detectImportOptions(filepath);
    opts.VariableNamingRule = 'preserve';
    tbl = readtable(filepath, opts);

    if isempty(tbl) || height(tbl) < 1
        error_msg = 'CSV file contains no data rows.';
        return;
    end

    var_names = tbl.Properties.VariableNames;
    var_lower = lower(strtrim(var_names));

    % Map column names flexibly
    col_id = find_col(var_lower, {'point_id', 'id', 'point'});
    col_x = find_col(var_lower, {'x', 'x_mm'});
    col_y = find_col(var_lower, {'y', 'y_mm'});
    col_z = find_col(var_lower, {'z', 'z_mm'});
    col_phi = find_col(var_lower, {'phi', 'phi_deg', 'phi_rad', 'pitch'});
    col_dur = find_col(var_lower, {'duration', 'duration_s', 'time', 'dt'});
    col_act = find_col(var_lower, {'action', 'type', 'command'});

    if isempty(col_x) || isempty(col_y) || isempty(col_z)
        error_msg = 'CSV missing required Cartesian coordinate columns (x/x_mm, y/y_mm, z/z_mm).';
        return;
    end

    n_rows = height(tbl);
    wp_list = empty_wp;

    for i = 1:n_rows
        % ID
        if ~isempty(col_id)
            v_id = tbl{i, col_id};
            if iscell(v_id), v_id = v_id{1}; end
            if ischar(v_id) || isstring(v_id), v_id = str2double(v_id); end
        else
            v_id = i;
        end

        % X, Y, Z
        v_x = tbl{i, col_x};
        v_y = tbl{i, col_y};
        v_z = tbl{i, col_z};
        if iscell(v_x), v_x = str2double(v_x{1}); end
        if iscell(v_y), v_y = str2double(v_y{1}); end
        if iscell(v_z), v_z = str2double(v_z{1}); end

        % Phi
        if ~isempty(col_phi)
            v_phi = tbl{i, col_phi};
            if iscell(v_phi), v_phi = str2double(v_phi{1}); end
        else
            v_phi = 0.0;
        end

        % Duration
        if ~isempty(col_dur)
            v_dur = tbl{i, col_dur};
            if iscell(v_dur), v_dur = str2double(v_dur{1}); end
        else
            v_dur = 2.0;
        end

        % Action
        if ~isempty(col_act)
            v_act = tbl{i, col_act};
            if iscell(v_act), v_act = v_act{1}; end
            v_act = upper(char(v_act));
        else
            v_act = 'MOVE';
        end

        wp_list(i).id = double(v_id);
        wp_list(i).x = double(v_x);
        wp_list(i).y = double(v_y);
        wp_list(i).z = double(v_z);
        wp_list(i).phi = double(v_phi);
        wp_list(i).duration = double(v_dur);
        wp_list(i).action = char(v_act);
    end

    waypoints = wp_list;
    is_valid = true;
end

%% Helper: Parse JSON
function [waypoints, is_valid, error_msg] = parse_json(filepath, empty_wp)
    waypoints = empty_wp;
    is_valid = false;
    error_msg = '';

    raw_text = fileread(filepath);
    if isempty(strtrim(raw_text))
        error_msg = 'JSON file is completely empty.';
        return;
    end

    data = jsondecode(raw_text);
    if ~isstruct(data)
        error_msg = 'JSON root must be an object.';
        return;
    end

    if ~isfield(data, 'waypoints')
        error_msg = 'JSON missing required "waypoints" field.';
        return;
    end

    raw_wp = data.waypoints;
    if isempty(raw_wp)
        error_msg = 'JSON "waypoints" array is empty.';
        return;
    end

    % Convert struct array or cell array
    if iscell(raw_wp)
        n_wp = numel(raw_wp);
        wp_list = empty_wp;
        for i = 1:n_wp
            item = raw_wp{i};
            wp_list(i) = extract_json_item(item, i);
        end
    elseif isstruct(raw_wp)
        n_wp = numel(raw_wp);
        wp_list = empty_wp;
        for i = 1:n_wp
            wp_list(i) = extract_json_item(raw_wp(i), i);
        end
    else
        error_msg = '"waypoints" must be an array of waypoint objects.';
        return;
    end

    waypoints = wp_list;
    is_valid = true;
end

%% Helper: Extract single JSON item
function item_wp = extract_json_item(item, default_id)
    fields = fieldnames(item);
    fields_lower = lower(fields);

    % ID
    col_id = find_col(fields_lower, {'id', 'point_id', 'point'});
    if ~isempty(col_id)
        v_id = double(item.(fields{col_id}));
    else
        v_id = double(default_id);
    end

    % X, Y, Z
    col_x = find_col(fields_lower, {'x', 'x_mm'});
    col_y = find_col(fields_lower, {'y', 'y_mm'});
    col_z = find_col(fields_lower, {'z', 'z_mm'});

    if ~isempty(col_x), v_x = double(item.(fields{col_x})); else, v_x = NaN; end
    if ~isempty(col_y), v_y = double(item.(fields{col_y})); else, v_y = NaN; end
    if ~isempty(col_z), v_z = double(item.(fields{col_z})); else, v_z = NaN; end

    % Phi
    col_phi = find_col(fields_lower, {'phi', 'phi_deg', 'phi_rad', 'pitch'});
    if ~isempty(col_phi)
        v_phi = double(item.(fields{col_phi}));
    else
        v_phi = 0.0;
    end

    % Duration
    col_dur = find_col(fields_lower, {'duration', 'duration_s', 'time', 'dt'});
    if ~isempty(col_dur)
        v_dur = double(item.(fields{col_dur}));
    else
        v_dur = 2.0;
    end

    % Action
    col_act = find_col(fields_lower, {'action', 'type', 'command'});
    if ~isempty(col_act)
        v_act = upper(char(item.(fields{col_act})));
    else
        v_act = 'MOVE';
    end

    item_wp.id = v_id;
    item_wp.x = v_x;
    item_wp.y = v_y;
    item_wp.z = v_z;
    item_wp.phi = v_phi;
    item_wp.duration = v_dur;
    item_wp.action = v_act;
end

%% Helper: Find column index by aliases
function idx = find_col(name_list, aliases)
    idx = [];
    for k = 1:numel(aliases)
        match = find(strcmpi(name_list, aliases{k}), 1);
        if ~isempty(match)
            idx = match;
            return;
        end
    end
end

%% Helper: Validate waypoints integrity and workspace bounds
function [is_valid, error_msg] = validate_waypoints(waypoints)
    is_valid = false;
    error_msg = '';

    if isempty(waypoints)
        error_msg = 'Trajectory waypoint list is empty.';
        return;
    end

    required_fields = {'id', 'x', 'y', 'z', 'phi', 'duration', 'action'};
    for k = 1:numel(required_fields)
        if ~isfield(waypoints, required_fields{k})
            error_msg = sprintf('Waypoints missing required field: %s', required_fields{k});
            return;
        end
    end

    for i = 1:numel(waypoints)
        wp = waypoints(i);

        % Type and NaN/Inf checks
        if ~isnumeric(wp.id) || isnan(wp.id) || isinf(wp.id)
            error_msg = sprintf('Waypoint %d: id must be a finite number.', i);
            return;
        end
        if ~isnumeric(wp.x) || isnan(wp.x) || isinf(wp.x)
            error_msg = sprintf('Waypoint %d: x coordinate is invalid or NaN.', i);
            return;
        end
        if ~isnumeric(wp.y) || isnan(wp.y) || isinf(wp.y)
            error_msg = sprintf('Waypoint %d: y coordinate is invalid or NaN.', i);
            return;
        end
        if ~isnumeric(wp.z) || isnan(wp.z) || isinf(wp.z)
            error_msg = sprintf('Waypoint %d: z coordinate is invalid or NaN.', i);
            return;
        end
        if ~isnumeric(wp.phi) || isnan(wp.phi) || isinf(wp.phi)
            error_msg = sprintf('Waypoint %d: phi angle is invalid or NaN.', i);
            return;
        end
        if ~isnumeric(wp.duration) || isnan(wp.duration) || wp.duration <= 0
            error_msg = sprintf('Waypoint %d: duration must be positive (> 0).', i);
            return;
        end
        if isempty(wp.action) || (~ischar(wp.action) && ~isstring(wp.action))
            error_msg = sprintf('Waypoint %d: action must be a non-empty string.', i);
            return;
        end

        % Physical workspace reachability check (myPalletizer 260 Pi max reach ~ 260..318 mm)
        r_xy = hypot(wp.x, wp.y);
        if r_xy > 330.0 || wp.z < -50.0 || wp.z > 400.0
            error_msg = sprintf('Waypoint %d [%.1f, %.1f, %.1f] exceeds maximum robot reach envelope.', ...
                wp.id, wp.x, wp.y, wp.z);
            return;
        end

        % Kinematic reachability check via inverse_kinematics if available
        if exist('inverse_kinematics', 'file')
            % Waypoints store phi in degrees
            phi_rad = deg2rad(wp.phi);
            [~, ik_success] = inverse_kinematics(wp.x, wp.y, wp.z, phi_rad);
            if ~ik_success && abs(wp.phi) <= 2*pi
                % Fallback: test if input was already in radians
                [~, ik_success] = inverse_kinematics(wp.x, wp.y, wp.z, wp.phi);
            end
            if ~ik_success
                error_msg = sprintf('Waypoint %d [%.1f, %.1f, %.1f] is kinematically unreachable.', ...
                    wp.id, wp.x, wp.y, wp.z);
                return;
            end
        end
    end

    is_valid = true;
end
