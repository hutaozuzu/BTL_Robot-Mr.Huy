function varargout = palletizer_gui(varargin)
% PALLETIZER_GUI Interactive 5-Panel Control, Kinematics Visualizer & Motion Analytics GUI
% for the 4-DOF myPalletizer 260 Pi robot manipulator.
%
% Panels:
%   Panel 1: 3D Robot Kinematics Visualizer (3D skeleton, frames, trail, pallet grid)
%   Panel 2: Manual Joint Sliders (J1..J4 limits, Zero and Home buttons)
%   Panel 3: Cartesian Jog Controller (X, Y, Z, Phi inputs, Solve IK, Reachability LED)
%   Panel 4: Trajectory File Importer & Runner (CSV/JSON loader, UITable, Speed 0.1x-2.0x, Run/Pause/Stop)
%   Panel 5: Motion Analytics Scope (4 subplots: angles, velocities with +-120 deg/s bounds, accelerations, tracking error)
%
% Syntax:
%   palletizer_gui()
%   fig = palletizer_gui('Visible', 'off')
%   [fig, handles] = palletizer_gui(...)
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Parse optional inputs (e.g. 'Visible', 'off')
    p = inputParser;
    addParameter(p, 'Visible', 'on', @(x) ismember(lower(x), {'on', 'off'}));
    addParameter(p, 'TrajectoryFile', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    init_visible = p.Results.Visible;
    init_file = char(p.Results.TrajectoryFile);

    % Ensure required directories are on path
    guiDir = fileparts(mfilename('fullpath'));
    if isempty(guiDir), guiDir = pwd; end
    projDir = fileparts(guiDir);
    addpath(fullfile(projDir, 'kinematics'));
    addpath(fullfile(projDir, 'trajectory'));
    addpath(fullfile(projDir, 'models'));

    % Load 3D CAD meshes
    cad_file = fullfile(projDir, 'models', 'robot_cad_meshes.mat');
    if exist(cad_file, 'file')
        cad_data = load(cad_file);
        cad_meshes = cad_data.cad_meshes;
    else
        cad_meshes = [];
    end

    % Color palette (modern engineering theme)
    c_bg = [0.93, 0.94, 0.96];
    c_pnl = [0.98, 0.98, 0.99];
    c_accent = [0.12, 0.35, 0.65];
    c_green = [0.15, 0.65, 0.25];
    c_red = [0.85, 0.20, 0.20];

    % Hardware limits (manufacturer specifications)
    % J1: [-162, 162] deg
    % J2: [-2, 90] deg
    % J3: [-92, 60] deg
    % J4: [-180, 180] deg
    limits_deg = [ ...
        -162.0, 162.0; ...
          -2.0,  90.0; ...
         -92.0,  60.0; ...
        -180.0, 180.0  ...
    ];
    standby_pose = [240.0, 0.0, 180.0, 0.0]; % mm and deg

    % Main Figure
    fig_w = 1320;
    fig_h = 780;
    screen_size = get(0, 'ScreenSize');
    fig_x = max(10, (screen_size(3) - fig_w) / 2);
    fig_y = max(30, (screen_size(4) - fig_h) / 2);

    h_fig = figure( ...
        'Name', 'myPalletizer 260 Pi — Interactive Control & Motion Analytics', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'ToolBar', 'figure', ...
        'Position', [fig_x, fig_y, fig_w, fig_h], ...
        'Color', c_bg, ...
        'Visible', init_visible, ...
        'CloseRequestFcn', @on_close_fig ...
    );

    % State structure
    state = struct();
    if exist('inverse_kinematics', 'file')
        [th_init, ~] = inverse_kinematics(standby_pose(1), standby_pose(2), standby_pose(3), deg2rad(standby_pose(4)));
    else
        th_init = [0.0, 0.4430, -0.6865, 0.2435];
    end
    state.current_theta = th_init;
    state.current_pose = standby_pose;
    state.waypoints = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, 'phi', {}, 'duration', {}, 'action', {});
    state.traj_struct = [];
    state.trail_pts = [];
    state.speed_factor = 1.0;
    state.is_running = false;
    state.is_paused = false;
    state.timer_idx = 1;
    state.anim_timer = [];

    handles = struct();
    handles.fig = h_fig;
    handles.limits_deg = limits_deg;

    %% ====================================================================
    %% PANEL 1: 3D Robot Kinematics Visualizer
    %% ====================================================================
    handles.panel1 = uipanel( ...
        'Parent', h_fig, ...
        'Title', ' 1. 3D Robot Kinematics Visualizer ', ...
        'FontWeight', 'bold', 'FontSize', 10, 'ForegroundColor', c_accent, ...
        'BackgroundColor', c_pnl, ...
        'Position', [0.012, 0.300, 0.360, 0.685] ...
    );

    handles.ax_3d = axes('Parent', handles.panel1, 'Position', [0.08, 0.08, 0.88, 0.88]);
    grid(handles.ax_3d, 'on');
    box(handles.ax_3d, 'on');
    axis(handles.ax_3d, 'equal');
    view(handles.ax_3d, 45, 25);
    xlabel(handles.ax_3d, 'X (mm)', 'FontWeight', 'bold');
    ylabel(handles.ax_3d, 'Y (mm)', 'FontWeight', 'bold');
    zlabel(handles.ax_3d, 'Z (mm)', 'FontWeight', 'bold');
    xlim(handles.ax_3d, [-200, 320]);
    ylim(handles.ax_3d, [-250, 250]);
    zlim(handles.ax_3d, [-20, 320]);
    hold(handles.ax_3d, 'on');

    % Workspace visual fixtures
    % 1. Robot base mount disc
    th_circ = linspace(0, 2*pi, 40);
    r_base = 50;
    patch(handles.ax_3d, r_base*cos(th_circ), r_base*sin(th_circ), zeros(size(th_circ)), ...
        [0.8, 0.8, 0.85], 'FaceAlpha', 0.8, 'EdgeColor', [0.4, 0.4, 0.5], 'DisplayName', 'Base Mount');

    % 2. Conveyor Pick Zone (X ~ 220, Y ~ -100, Z ~ 0..30)
    conveyor_x = [190, 250, 250, 190];
    conveyor_y = [-130, -130, -70, -70];
    patch(handles.ax_3d, conveyor_x, conveyor_y, zeros(1, 4), ...
        [0.85, 0.75, 0.65], 'FaceAlpha', 0.6, 'EdgeColor', [0.6, 0.5, 0.4], 'DisplayName', 'Conveyor Pick');
    text(handles.ax_3d, 220, -100, 10, 'Conveyor', 'HorizontalAlignment', 'center', ...
        'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.4, 0.3, 0.2]);

    % 3. Pallet Drop Zone (X ~ 180, Y ~ 160, Z ~ 0..30)
    pallet_x = [150, 210, 210, 150];
    pallet_y = [130, 130, 190, 190];
    patch(handles.ax_3d, pallet_x, pallet_y, zeros(1, 4), ...
        [0.70, 0.85, 0.70], 'FaceAlpha', 0.6, 'EdgeColor', [0.3, 0.6, 0.3], 'DisplayName', 'Pallet Drop');
    text(handles.ax_3d, 180, 160, 10, 'Pallet', 'HorizontalAlignment', 'center', ...
        'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.2, 0.5, 0.2]);

    % 4. Trail line (motion path history)
    handles.h_trail = plot3(handles.ax_3d, NaN, NaN, NaN, ...
        '--', 'Color', [0.15, 0.75, 0.35], 'LineWidth', 1.8, 'DisplayName', 'TCP Trail');

    % 5. Waypoints scatter markers
    handles.h_waypoints = scatter3(handles.ax_3d, NaN, NaN, NaN, 7, ...
        'filled', 'MarkerFaceColor', [0.85, 0.2, 0.6], 'MarkerEdgeColor', 'none', 'DisplayName', 'Waypoints');

    % 6. Robot skeleton links line (reference axis)
    handles.h_links = plot3(handles.ax_3d, [0 0], [0 0], [0 0], ...
        ':', 'Color', [0.2, 0.4, 0.7, 0.4], 'LineWidth', 1.5, 'DisplayName', 'Kinematic Axis');

    % 7. Robot joint spheres
    handles.h_joints = scatter3(handles.ax_3d, zeros(1, 5), zeros(1, 5), zeros(1, 5), 40, ...
        'filled', 'MarkerFaceColor', [0.92, 0.40, 0.15], 'MarkerEdgeColor', 'none', 'DisplayName', 'Joints');

    % 8. Tool pointer / orientation frame
    handles.h_tool = plot3(handles.ax_3d, [0 0], [0 0], [0 0], ...
        '-', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 2.5, 'DisplayName', 'Tool Center Point');

    % 9. Real 3D CAD Solid Mesh Patches (myPalletizer 260 Pi)
    mesh_colors = { ...
        [0.88, 0.88, 0.90], ... % base: silver white
        [0.22, 0.25, 0.32], ... % link1: dark charcoal
        [0.94, 0.94, 0.96], ... % link2: white arm
        [0.94, 0.94, 0.96], ... % link3: white forearm
        [0.22, 0.25, 0.32], ... % link4: dark wrist
        [0.85, 0.45, 0.15]  ... % link5: orange tool flange
    };
    link_names = {'base', 'link1', 'link2', 'link3', 'link4', 'link5'};
    handles.h_cad = gobjects(1, 6);
    if ~isempty(cad_meshes)
        for i = 1:6
            nm = link_names{i};
            handles.h_cad(i) = patch('Parent', handles.ax_3d, ...
                'Faces', cad_meshes.(nm).faces, ...
                'Vertices', cad_meshes.(nm).vertices, ...
                'FaceColor', mesh_colors{i}, ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 1.0, ...
                'SpecularStrength', 0.3, ...
                'DiffuseStrength', 0.8, ...
                'AmbientStrength', 0.4, ...
                'DisplayName', ['CAD ' nm]);
        end
        camlight(handles.ax_3d, 'headlight');
        lighting(handles.ax_3d, 'gouraud');
    end

    %% ====================================================================
    %% PANEL 2: Manual Joint Sliders
    %% ====================================================================
    handles.panel2 = uipanel( ...
        'Parent', h_fig, ...
        'Title', ' 2. Manual Joint Sliders ', ...
        'FontWeight', 'bold', 'FontSize', 10, 'ForegroundColor', c_accent, ...
        'BackgroundColor', c_pnl, ...
        'Position', [0.012, 0.015, 0.360, 0.275] ...
    );

    % 4 Joint Sliders
    joint_names = {'J1 (Base Yaw)', 'J2 (Shoulder)', 'J3 (Elbow)', 'J4 (Wrist)'};
    handles.sliders = gobjects(4, 1);
    handles.lbl_vals = gobjects(4, 1);

    for j = 1:4
        y_pos = 0.72 - (j - 1) * 0.16;
        uicontrol('Parent', handles.panel2, 'Style', 'text', ...
            'String', sprintf('%s [%.0f..%.0f]°', joint_names{j}, limits_deg(j, 1), limits_deg(j, 2)), ...
            'Units', 'normalized', 'Position', [0.03, y_pos, 0.42, 0.11], ...
            'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 8, ...
            'BackgroundColor', c_pnl);

        range_j = limits_deg(j, 2) - limits_deg(j, 1);
        handles.sliders(j) = uicontrol('Parent', handles.panel2, 'Style', 'slider', ...
            'Min', limits_deg(j, 1), 'Max', limits_deg(j, 2), ...
            'Value', rad2deg(state.current_theta(j)), ...
            'SliderStep', [1.0 / range_j, 10.0 / range_j], ...
            'Units', 'normalized', 'Position', [0.46, y_pos, 0.36, 0.11], ...
            'Callback', @on_slider_change);

        handles.lbl_vals(j) = uicontrol('Parent', handles.panel2, 'Style', 'text', ...
            'String', sprintf('%.1f°', rad2deg(state.current_theta(j))), ...
            'Units', 'normalized', 'Position', [0.83, y_pos, 0.14, 0.11], ...
            'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'FontSize', 8, ...
            'BackgroundColor', c_pnl);
    end

    % Presets: Zero Position and Home Position
    handles.btn_zero = uicontrol('Parent', handles.panel2, 'Style', 'pushbutton', ...
        'String', 'Zero [0,0,0,0]', ...
        'Units', 'normalized', 'Position', [0.06, 0.04, 0.42, 0.12], ...
        'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', [0.88, 0.90, 0.94], ...
        'Callback', @on_btn_zero);

    handles.btn_home = uicontrol('Parent', handles.panel2, 'Style', 'pushbutton', ...
        'String', 'Home [Standby]', ...
        'Units', 'normalized', 'Position', [0.52, 0.04, 0.42, 0.12], ...
        'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', [0.85, 0.92, 0.88], ...
        'Callback', @on_btn_home);

    %% ====================================================================
    %% PANEL 3: Cartesian Jog Controller
    %% ====================================================================
    handles.panel3 = uipanel( ...
        'Parent', h_fig, ...
        'Title', ' 3. Cartesian Jog Controller ', ...
        'FontWeight', 'bold', 'FontSize', 10, 'ForegroundColor', c_accent, ...
        'BackgroundColor', c_pnl, ...
        'Position', [0.382, 0.655, 0.285, 0.330] ...
    );

    coord_labels = {'X (mm):', 'Y (mm):', 'Z (mm):', 'Phi (°):'};
    handles.edit_coords = gobjects(4, 1);
    handles.btn_jog_minus = gobjects(4, 1);
    handles.btn_jog_plus = gobjects(4, 1);
    step_sizes = [10.0, 10.0, 10.0, 5.0];

    for c = 1:4
        y_pos = 0.78 - (c - 1) * 0.15;

        uicontrol('Parent', handles.panel3, 'Style', 'text', ...
            'String', coord_labels{c}, ...
            'Units', 'normalized', 'Position', [0.04, y_pos, 0.24, 0.10], ...
            'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 8, ...
            'BackgroundColor', c_pnl);

        % Minus step button
        handles.btn_jog_minus(c) = uicontrol('Parent', handles.panel3, 'Style', 'pushbutton', ...
            'String', sprintf('-%g', step_sizes(c)), ...
            'Tag', sprintf('btn_jog_minus_%d', c), ...
            'Units', 'normalized', 'Position', [0.30, y_pos, 0.15, 0.10], ...
            'FontSize', 8, 'FontWeight', 'bold', ...
            'Callback', @(src, ev) on_step_jog(c, -step_sizes(c)));

        % Coordinate Edit box
        handles.edit_coords(c) = uicontrol('Parent', handles.panel3, 'Style', 'edit', ...
            'String', sprintf('%.1f', standby_pose(c)), ...
            'Units', 'normalized', 'Position', [0.47, y_pos, 0.28, 0.10], ...
            'FontSize', 8, 'FontWeight', 'bold', 'BackgroundColor', 'w', ...
            'Callback', @on_cartesian_edit);

        % Plus step button
        handles.btn_jog_plus(c) = uicontrol('Parent', handles.panel3, 'Style', 'pushbutton', ...
            'String', sprintf('+%g', step_sizes(c)), ...
            'Tag', sprintf('btn_jog_plus_%d', c), ...
            'Units', 'normalized', 'Position', [0.77, y_pos, 0.15, 0.10], ...
            'FontSize', 8, 'FontWeight', 'bold', ...
            'Callback', @(src, ev) on_step_jog(c, +step_sizes(c)));
    end

    % Solve IK button
    handles.btn_ik = uicontrol('Parent', handles.panel3, 'Style', 'pushbutton', ...
        'String', 'Solve Inverse Kinematics (IK)', ...
        'Units', 'normalized', 'Position', [0.06, 0.17, 0.88, 0.12], ...
        'FontWeight', 'bold', 'FontSize', 9, ...
        'BackgroundColor', [0.82, 0.88, 0.96], ...
        'Callback', @(src, ev) on_solve_ik());

    % Reachability LED indicator
    handles.led_status = uicontrol('Parent', handles.panel3, 'Style', 'text', ...
        'String', '● REACHABLE (Standby)', ...
        'Units', 'normalized', 'Position', [0.06, 0.04, 0.88, 0.10], ...
        'FontWeight', 'bold', 'FontSize', 9, ...
        'ForegroundColor', 'w', 'BackgroundColor', c_green);

    %% ====================================================================
    %% PANEL 4: Trajectory File Importer & Runner
    %% ====================================================================
    handles.panel4 = uipanel( ...
        'Parent', h_fig, ...
        'Title', ' 4. Trajectory File Importer & Runner ', ...
        'FontWeight', 'bold', 'FontSize', 10, 'ForegroundColor', c_accent, ...
        'BackgroundColor', c_pnl, ...
        'Position', [0.382, 0.015, 0.285, 0.630] ...
    );

    % File path edit and browse
    uicontrol('Parent', handles.panel4, 'Style', 'text', ...
        'String', 'Trajectory File Path (CSV / JSON):', ...
        'Units', 'normalized', 'Position', [0.03, 0.94, 0.94, 0.045], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', c_pnl);

    default_csv = fullfile(pwd, '..', 'data', 'trajectory_sample.csv');
    if ~exist(default_csv, 'file')
        default_csv = 'd:\Lidar\myPalletizer_BTL\data\trajectory_sample.csv';
    end

    handles.edit_filepath = uicontrol('Parent', handles.panel4, 'Style', 'edit', ...
        'String', default_csv, ...
        'Units', 'normalized', 'Position', [0.03, 0.88, 0.68, 0.055], ...
        'FontSize', 8, 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');

    handles.btn_browse = uicontrol('Parent', handles.panel4, 'Style', 'pushbutton', ...
        'String', 'Browse...', ...
        'Units', 'normalized', 'Position', [0.73, 0.88, 0.24, 0.055], ...
        'FontSize', 8, 'FontWeight', 'bold', ...
        'Callback', @on_browse_file);

    handles.btn_load = uicontrol('Parent', handles.panel4, 'Style', 'pushbutton', ...
        'String', 'Import & Validate Trajectory File', ...
        'Units', 'normalized', 'Position', [0.03, 0.815, 0.94, 0.055], ...
        'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.85, 0.90, 0.98], ...
        'Callback', @(src, ev) on_load_file());

    % UITable of Waypoints
    col_names = {'ID', 'X (mm)', 'Y (mm)', 'Z (mm)', 'Phi (°)', 'Duration (s)', 'Action'};
    handles.table_waypoints = uitable('Parent', handles.panel4, ...
        'ColumnName', col_names, ...
        'ColumnFormat', {'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'char'}, ...
        'ColumnEditable', [false, false, false, false, false, false, false], ...
        'Units', 'normalized', 'Position', [0.03, 0.28, 0.94, 0.52], ...
        'RowName', []);

    % Speed factor slider
    uicontrol('Parent', handles.panel4, 'Style', 'text', ...
        'String', 'Speed Scaling (0.1x - 2.0x):', ...
        'Units', 'normalized', 'Position', [0.03, 0.215, 0.55, 0.045], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', c_pnl);

    handles.lbl_speed = uicontrol('Parent', handles.panel4, 'Style', 'text', ...
        'String', '1.0x', ...
        'Units', 'normalized', 'Position', [0.60, 0.215, 0.37, 0.045], ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', c_pnl);

    handles.slider_speed = uicontrol('Parent', handles.panel4, 'Style', 'slider', ...
        'Min', 0.1, 'Max', 2.0, 'Value', 1.0, ...
        'SliderStep', [0.05 / 1.9, 0.2 / 1.9], ...
        'Units', 'normalized', 'Position', [0.03, 0.165, 0.94, 0.045], ...
        'Callback', @on_speed_change);

    % Execution Buttons: Run, Pause, Stop
    handles.btn_run = uicontrol('Parent', handles.panel4, 'Style', 'pushbutton', ...
        'String', '▶ Run', ...
        'Units', 'normalized', 'Position', [0.03, 0.085, 0.30, 0.065], ...
        'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.75, 0.92, 0.75], ...
        'Callback', @(src, ev) on_run_trajectory());

    handles.btn_pause = uicontrol('Parent', handles.panel4, 'Style', 'pushbutton', ...
        'String', '❚❚ Pause', ...
        'Units', 'normalized', 'Position', [0.35, 0.085, 0.30, 0.065], ...
        'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.95, 0.92, 0.75], ...
        'Callback', @(src, ev) on_pause_trajectory());

    handles.btn_stop = uicontrol('Parent', handles.panel4, 'Style', 'pushbutton', ...
        'String', '■ Stop', ...
        'Units', 'normalized', 'Position', [0.67, 0.085, 0.30, 0.065], ...
        'FontSize', 9, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.95, 0.80, 0.80], ...
        'Callback', @(src, ev) on_stop_trajectory());

    % Execution status label
    handles.lbl_run_status = uicontrol('Parent', handles.panel4, 'Style', 'text', ...
        'String', 'Status: Ready to import trajectory.', ...
        'Units', 'normalized', 'Position', [0.03, 0.015, 0.94, 0.055], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 8, ...
        'BackgroundColor', c_pnl, 'ForegroundColor', [0.2, 0.2, 0.2]);

    %% ====================================================================
    %% PANEL 5: Motion Analytics Scope (4 Subplots)
    %% ====================================================================
    handles.panel5 = uipanel( ...
        'Parent', h_fig, ...
        'Title', ' 5. Motion Analytics Scope ', ...
        'FontWeight', 'bold', 'FontSize', 10, 'ForegroundColor', c_accent, ...
        'BackgroundColor', c_pnl, ...
        'Position', [0.677, 0.015, 0.312, 0.970] ...
    );

    % 4 subplots vertically stacked
    handles.ax_scope1 = axes('Parent', handles.panel5, 'Position', [0.15, 0.78, 0.80, 0.17]);
    handles.ax_scope2 = axes('Parent', handles.panel5, 'Position', [0.15, 0.54, 0.80, 0.17]);
    handles.ax_scope3 = axes('Parent', handles.panel5, 'Position', [0.15, 0.30, 0.80, 0.17]);
    handles.ax_scope4 = axes('Parent', handles.panel5, 'Position', [0.15, 0.06, 0.80, 0.17]);

    setup_scope_axes();

    %% Expose programmatic callbacks and methods on handles struct
    handles.update_robot = @update_robot_state;
    handles.on_slider_change = @on_slider_change;
    handles.on_solve_ik = @on_solve_ik;
    handles.load_file = @load_trajectory_file;
    handles.run_trajectory = @on_run_trajectory;
    handles.stop_trajectory = @on_stop_trajectory;
    handles.export_metrics = @export_motion_metrics;
    handles.jog_step = @(c, step) on_step_jog(c, step);
    handles.cleanup = @cleanup_resources;

    % Store in guidata
    guidata(h_fig, handles);

    % Initial robot 3D render at standby
    update_robot_state(state.current_theta);

    % Auto-load default trajectory file if present
    if ~isempty(init_file) && exist(init_file, 'file')
        load_trajectory_file(init_file);
    elseif exist(default_csv, 'file')
        load_trajectory_file(default_csv);
    end

    if nargout > 0
        varargout{1} = h_fig;
    end
    if nargout > 1
        varargout{2} = handles;
    end

    %% ====================================================================
    %% Nested Helper Functions
    %% ====================================================================

    function setup_scope_axes()
        % Subplot 1: Joint Angles
        cla(handles.ax_scope1);
        grid(handles.ax_scope1, 'on');
        title(handles.ax_scope1, 'Joint Positions \theta(t)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope1, 'Angle (deg)', 'FontSize', 7);

        % Subplot 2: Joint Velocities with +-120 deg/s threshold lines
        cla(handles.ax_scope2);
        grid(handles.ax_scope2, 'on');
        hold(handles.ax_scope2, 'on');
        title(handles.ax_scope2, 'Joint Velocities \omega(t) & Limits (±120 °/s)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope2, 'Vel (°/s)', 'FontSize', 7);
        ylim(handles.ax_scope2, [-150, 150]);
        % Visual threshold lines
        yline(handles.ax_scope2, 120.0, 'r--', 'LineWidth', 1.2, 'DisplayName', '+120 °/s limit');
        yline(handles.ax_scope2, -120.0, 'r--', 'LineWidth', 1.2, 'DisplayName', '-120 °/s limit');

        % Subplot 3: Joint Accelerations
        cla(handles.ax_scope3);
        grid(handles.ax_scope3, 'on');
        title(handles.ax_scope3, 'Joint Accelerations \alpha(t)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope3, 'Accel (°/s^2)', 'FontSize', 7);

        % Subplot 4: Tracking Error Norm
        cla(handles.ax_scope4);
        grid(handles.ax_scope4, 'on');
        title(handles.ax_scope4, 'Cartesian Tracking Error Norm ||P_{act} - P_{des}||', 'FontSize', 8, 'FontWeight', 'bold');
        xlabel(handles.ax_scope4, 'Time (s)', 'FontSize', 7);
        ylabel(handles.ax_scope4, 'Error (mm)', 'FontSize', 7);
    end

    function update_robot_state(theta_rad)
        state.current_theta = reshape(theta_rad, 1, 4);

        % Calculate Forward Kinematics
        if exist('forward_kinematics', 'file')
            [pose, T_all] = forward_kinematics(state.current_theta, 'rad');
        else
            % Fallback pure native FK if not on path
            d1 = 120.0; a2 = 140.0; a3 = 134.0; a4 = 44.0;
            th = state.current_theta;
            T01 = [cos(th(1)) 0 sin(th(1)) 0; sin(th(1)) 0 -cos(th(1)) 0; 0 1 0 d1; 0 0 0 1];
            T12 = [cos(th(2)) -sin(th(2)) 0 a2*cos(th(2)); sin(th(2)) cos(th(2)) 0 a2*sin(th(2)); 0 0 1 0; 0 0 0 1];
            T23 = [cos(th(3)) -sin(th(3)) 0 a3*cos(th(3)); sin(th(3)) cos(th(3)) 0 a3*sin(th(3)); 0 0 1 0; 0 0 0 1];
            T34 = [cos(th(4)) -sin(th(4)) 0 a4*cos(th(4)); sin(th(4)) cos(th(4)) 0 a4*sin(th(4)); 0 0 1 0; 0 0 0 1];
            T02 = T01 * T12;
            T03 = T02 * T23;
            T04 = T03 * T34;
            T_all = {T01, T02, T03, T04};
            pose = [T04(1:3, 4)', th(2) + th(3) + th(4)];
        end

        state.current_pose = [pose(1:3), rad2deg(pose(4))];

        % Extract joint 3D points
        p0 = [0.0, 0.0, 0.0];
        p1 = T_all{1}(1:3, 4)';
        p2 = T_all{2}(1:3, 4)';
        p3 = T_all{3}(1:3, 4)';
        p4 = T_all{4}(1:3, 4)';
        pts = [p0; p1; p2; p3; p4];

        % Update 3D skeleton graphic line and joint spheres
        if isvalid(handles.h_links)
            set(handles.h_links, 'XData', pts(:, 1), 'YData', pts(:, 2), 'ZData', pts(:, 3));
        end
        if isvalid(handles.h_joints)
            set(handles.h_joints, 'XData', pts(:, 1), 'YData', pts(:, 2), 'ZData', pts(:, 3));
        end

        % Update tool orientation vector (30 mm extension in tool pitch direction)
        tool_dir = [cos(state.current_theta(1))*cos(pose(4)), ...
                    sin(state.current_theta(1))*cos(pose(4)), ...
                    sin(pose(4))] * 30.0;
        p_tool = p4 + tool_dir;
        if isvalid(handles.h_tool)
            set(handles.h_tool, 'XData', [p4(1), p_tool(1)], ...
                               'YData', [p4(2), p_tool(2)], ...
                               'ZData', [p4(3), p_tool(3)]);
        end

        % Update Real 3D CAD Solid Meshes
        if ~isempty(cad_meshes)
            th1 = state.current_theta(1);
            th2_u = pi/2 - state.current_theta(2);
            th3_u = -pi/2 - state.current_theta(3);
            th4_u = state.current_theta(4);

            T_b = eye(4);
            T_1 = [cos(th1), -sin(th1), 0, 0; sin(th1), cos(th1), 0, 0; 0, 0, 1, 120; 0, 0, 0, 1];
            T_2 = T_1 * [1, 0, 0, 0; 0, 0, 1, 0; 0, -1, 0, 0; 0, 0, 0, 1] * ...
                        [cos(th2_u), -sin(th2_u), 0, 0; sin(th2_u), cos(th2_u), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
            T_3 = T_2 * [cos(th3_u), -sin(th3_u), 0, 0; sin(th3_u), cos(th3_u), 0, -140; 0, 0, 1, 0; 0, 0, 0, 1];
            T_4 = T_3 * [1, 0, 0, 134; 0, -1, 0, 0; 0, 0, -1, 0; 0, 0, 0, 1];
            T_5 = T_4 * [cos(th4_u), -sin(th4_u), 0, 0; sin(th4_u), cos(th4_u), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];

            T_cad = {T_b, T_1, T_2, T_3, T_4, T_5};
            for m_i = 1:6
                if isvalid(handles.h_cad(m_i))
                    nm = link_names{m_i};
                    V0 = cad_meshes.(nm).vertices;
                    V_w = [V0, ones(size(V0, 1), 1)] * T_cad{m_i}';
                    set(handles.h_cad(m_i), 'Vertices', V_w(:, 1:3));
                end
            end
        end

        % Update Cartesian Edit boxes in Panel 3
        for k = 1:4
            if isvalid(handles.edit_coords(k))
                set(handles.edit_coords(k), 'String', sprintf('%.1f', state.current_pose(k)));
            end
        end

        % Update Slider values and labels in Panel 2
        for k = 1:4
            val_deg = rad2deg(state.current_theta(k));
            if isvalid(handles.sliders(k))
                % Clamp slider value to bounds
                val_clamped = min(max(val_deg, limits_deg(k, 1)), limits_deg(k, 2));
                set(handles.sliders(k), 'Value', val_clamped);
            end
            if isvalid(handles.lbl_vals(k))
                set(handles.lbl_vals(k), 'String', sprintf('%.1f°', val_deg));
            end
        end
    end

    %% Callback: Sliders change
    function on_slider_change(~, ~)
        new_th_deg = zeros(1, 4);
        for k = 1:4
            new_th_deg(k) = get(handles.sliders(k), 'Value');
        end
        update_robot_state(deg2rad(new_th_deg));
        set(handles.led_status, 'String', '● MANUAL JOG (FK Updated)', ...
            'BackgroundColor', [0.2, 0.5, 0.8]);
    end

    %% Callback: Zero Button
    function on_btn_zero(~, ~)
        update_robot_state([0.0, 0.0, 0.0, 0.0]);
        set(handles.led_status, 'String', '● ZERO CONFIGURATION [0, 0, 0, 0]', ...
            'BackgroundColor', c_green);
    end

    %% Callback: Home Button
    function on_btn_home(~, ~)
        % Standby pose [240, 0, 180, 0] mm
        target = standby_pose;
        if exist('inverse_kinematics', 'file')
            [th_opt, success] = inverse_kinematics(target(1), target(2), target(3), deg2rad(target(4)));
        else
            th_opt = [0.0, 0.4430, -0.6865, 0.2435];
            success = true;
        end
        if success
            update_robot_state(th_opt);
            set(handles.led_status, 'String', '● HOME (Standby Restored)', ...
                'BackgroundColor', c_green);
        end
    end

    %% Callback: Step jog buttons (+-10 mm, +-5 deg)
    function on_step_jog(coord_idx, step_val)
        curr_val = str2double(get(handles.edit_coords(coord_idx), 'String'));
        if isnan(curr_val), curr_val = state.current_pose(coord_idx); end
        new_val = curr_val + step_val;
        set(handles.edit_coords(coord_idx), 'String', sprintf('%.1f', new_val));
        on_solve_ik();
    end

    %% Callback: Cartesian text edit
    function on_cartesian_edit(~, ~)
        on_solve_ik();
    end

    %% Callback: Solve IK
    function success = on_solve_ik()
        success = false;
        req_x = str2double(get(handles.edit_coords(1), 'String'));
        req_y = str2double(get(handles.edit_coords(2), 'String'));
        req_z = str2double(get(handles.edit_coords(3), 'String'));
        req_phi = str2double(get(handles.edit_coords(4), 'String'));

        if isnan(req_x) || isnan(req_y) || isnan(req_z) || isnan(req_phi)
            set(handles.led_status, 'String', '● INVALID INPUT (NaN)', ...
                'BackgroundColor', c_red);
            return;
        end

        phi_rad = deg2rad(req_phi);

        if exist('inverse_kinematics', 'file')
            [th_sol, ik_ok] = inverse_kinematics(req_x, req_y, req_z, phi_rad, 'closest', state.current_theta);
        else
            ik_ok = false;
            th_sol = [];
        end

        if ik_ok && ~isempty(th_sol)
            success = true;
            update_robot_state(th_sol);
            set(handles.led_status, 'String', '● REACHABLE (IK Solved)', ...
                'BackgroundColor', c_green);
        else
            set(handles.led_status, 'String', '● UNREACHABLE (Out of Envelope)', ...
                'BackgroundColor', c_red);
        end
    end

    %% Callback: Browse File
    function on_browse_file(~, ~)
        [filename, pathname] = uigetfile( ...
            {'*.csv;*.json', 'Trajectory Files (*.csv, *.json)'; ...
             '*.csv', 'CSV Waypoints (*.csv)'; ...
             '*.json', 'JSON Trajectory (*.json)'}, ...
            'Select Trajectory File');
        if isequal(filename, 0) || isequal(pathname, 0)
            return;
        end
        full_path = fullfile(pathname, filename);
        set(handles.edit_filepath, 'String', full_path);
        load_trajectory_file(full_path);
    end

    %% Callback: Load trajectory file
    function on_load_file()
        filepath = strtrim(get(handles.edit_filepath, 'String'));
        load_trajectory_file(filepath);
    end

    function [is_valid, msg] = load_trajectory_file(filepath)
        is_valid = false;
        msg = '';

        if isempty(filepath) || ~exist(filepath, 'file')
            msg = sprintf('File not found: %s', filepath);
            set(handles.lbl_run_status, 'String', ['Error: ' msg], 'ForegroundColor', c_red);
            return;
        end

        % Invoke trajectory_parser
        if exist('trajectory_parser', 'file')
            [wp, is_valid, err] = trajectory_parser(filepath);
        else
            is_valid = false;
            err = 'trajectory_parser.m not found on path.';
            wp = [];
        end

        if ~is_valid || isempty(wp)
            msg = err;
            set(handles.lbl_run_status, 'String', ['Parser Error: ' err], 'ForegroundColor', c_red);
            return;
        end

        state.waypoints = wp;
        n_wp = numel(wp);

        % Populate UITable
        table_data = cell(n_wp, 7);
        wp_xyz = zeros(n_wp, 3);
        for i = 1:n_wp
            table_data{i, 1} = wp(i).id;
            table_data{i, 2} = wp(i).x;
            table_data{i, 3} = wp(i).y;
            table_data{i, 4} = wp(i).z;
            table_data{i, 5} = wp(i).phi;
            table_data{i, 6} = wp(i).duration;
            table_data{i, 7} = wp(i).action;
            wp_xyz(i, :) = [wp(i).x, wp(i).y, wp(i).z];
        end
        set(handles.table_waypoints, 'Data', table_data);

        % Update Waypoints in 3D Plot
        if isvalid(handles.h_waypoints)
            set(handles.h_waypoints, 'XData', wp_xyz(:, 1), 'YData', wp_xyz(:, 2), 'ZData', wp_xyz(:, 3), 'SizeData', 7);
        end
        if isvalid(handles.h_trail)
            set(handles.h_trail, 'XData', wp_xyz(:, 1), 'YData', wp_xyz(:, 2), 'ZData', wp_xyz(:, 3));
        end

        [~, fname, fext] = fileparts(filepath);
        msg = sprintf('Successfully loaded %d waypoints from %s%s', n_wp, fname, fext);
        set(handles.lbl_run_status, 'String', msg, 'ForegroundColor', [0.1, 0.5, 0.1]);
    end

    %% Callback: Speed factor slider
    function on_speed_change(~, ~)
        val = get(handles.slider_speed, 'Value');
        state.speed_factor = val;
        set(handles.lbl_speed, 'String', sprintf('%.2fx', val));
    end

    %% Callback: Run trajectory
    function on_run_trajectory()
        if isempty(state.waypoints)
            set(handles.lbl_run_status, 'String', 'Error: No waypoints loaded. Click Import first.', ...
                'ForegroundColor', c_red);
            return;
        end

        wp = state.waypoints;
        n_wp = numel(wp);
        cart_wp = zeros(n_wp, 4);
        durations = zeros(n_wp - 1, 1);
        for i = 1:n_wp
            cart_wp(i, :) = [wp(i).x, wp(i).y, wp(i).z, deg2rad(wp(i).phi)];
            if i < n_wp
                durations(i) = wp(i).duration;
            end
        end

        % Scale segment durations by speed factor
        scaled_durations = max(0.2, durations / max(0.1, state.speed_factor));

        % Multi-point trajectory planner
        dt = 0.05;
        if exist('multi_point_planner', 'file')
            traj = multi_point_planner(cart_wp, scaled_durations, dt, 'cartesian');
        else
            % Fallback quintic interpolation
            traj = generate_fallback_trajectory(cart_wp, scaled_durations, dt);
        end

        state.traj_struct = traj;
        state.is_running = true;
        state.is_paused = false;
        state.timer_idx = 1;
        state.trail_pts = [];

        % Plot 4 Scope signals
        plot_motion_scope(traj);

        % If figure is hidden (headless unit testing), step to completion synchronously
        if strcmpi(get(h_fig, 'Visible'), 'off')
            update_robot_state(traj.theta(end, :));
            state.is_running = false;
            set(handles.lbl_run_status, 'String', 'Trajectory execution finished (Headless).', ...
                'ForegroundColor', [0.1, 0.5, 0.1]);
            return;
        end

        % Interactive mode: run animation timer
        stop_animation_timer();
        state.anim_timer = timer( ...
            'ExecutionMode', 'fixedRate', ...
            'Period', 0.04, ... % 25 Hz update
            'TimerFcn', @on_timer_step, ...
            'StopFcn', @(s, e) set(handles.lbl_run_status, 'String', 'Trajectory complete.') ...
        );
        start(state.anim_timer);
        set(handles.lbl_run_status, 'String', 'Executing trajectory...', 'ForegroundColor', c_accent);
    end

    %% Timer Step for Animation
    function on_timer_step(~, ~)
        if ~state.is_running || state.is_paused || isempty(state.traj_struct)
            return;
        end
        idx = state.timer_idx;
        traj = state.traj_struct;
        n_steps = size(traj.theta, 1);

        if idx > n_steps
            update_robot_state(traj.theta(end, :));
            stop_animation_timer();
            state.is_running = false;
            set(handles.lbl_run_status, 'String', 'Trajectory execution complete.', 'ForegroundColor', c_green);
            return;
        end

        % Advance robot joint posture
        th_k = traj.theta(idx, :);
        update_robot_state(th_k);

        % Update trail in 3D plot
        if isfield(traj, 'xyz_actual') && ~isempty(traj.xyz_actual)
            pt_k = traj.xyz_actual(idx, :);
        else
            pt_k = state.current_pose(1:3);
        end
        state.trail_pts = [state.trail_pts; pt_k];
        if isvalid(handles.h_trail)
            set(handles.h_trail, 'XData', state.trail_pts(:, 1), ...
                                'YData', state.trail_pts(:, 2), ...
                                'ZData', state.trail_pts(:, 3));
        end

        % Advance index (skip frames if needed for real-time sync)
        state.timer_idx = idx + 2;
    end

    %% Callback: Pause
    function on_pause_trajectory()
        if state.is_running
            state.is_paused = ~state.is_paused;
            if state.is_paused
                set(handles.lbl_run_status, 'String', 'Execution PAUSED.', 'ForegroundColor', [0.8, 0.5, 0.1]);
            else
                set(handles.lbl_run_status, 'String', 'Execution RESUMED.', 'ForegroundColor', c_accent);
            end
        end
    end

    %% Callback: Stop
    function on_stop_trajectory()
        stop_animation_timer();
        state.is_running = false;
        state.is_paused = false;
        state.timer_idx = 1;
        set(handles.lbl_run_status, 'String', 'Trajectory STOPPED.', 'ForegroundColor', c_red);
    end

    function stop_animation_timer()
        if ~isempty(state.anim_timer) && isvalid(state.anim_timer)
            stop(state.anim_timer);
            delete(state.anim_timer);
            state.anim_timer = [];
        end
    end

    %% Plot Analytics Scope
    function plot_motion_scope(traj)
        t = traj.t;
        th_deg = rad2deg(traj.theta);
        dth_deg = rad2deg(traj.dtheta);
        ddth_deg = rad2deg(traj.ddtheta);

        % Calculate Tracking Error Norm ||P_act - P_des||
        if isfield(traj, 'xyz_actual') && ~isempty(traj.xyz_actual)
            % Compute nominal desired from linear Cartesian interpolation or joint FK
            err_norm = zeros(numel(t), 1);
            for i = 1:numel(t)
                [p_act, ~] = forward_kinematics(traj.theta(i, :), 'rad');
                % Desired pose
                err_norm(i) = norm(p_act(1:3) - traj.xyz_actual(i, :));
            end
        else
            err_norm = zeros(numel(t), 1);
        end

        % Subplot 1: Joint Positions
        cla(handles.ax_scope1);
        grid(handles.ax_scope1, 'on');
        hold(handles.ax_scope1, 'on');
        plot(handles.ax_scope1, t, th_deg(:, 1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'J1');
        plot(handles.ax_scope1, t, th_deg(:, 2), 'Color', [0.85, 0.35, 0.1], 'LineWidth', 1.2, 'DisplayName', 'J2');
        plot(handles.ax_scope1, t, th_deg(:, 3), 'Color', [0.15, 0.70, 0.25], 'LineWidth', 1.2, 'DisplayName', 'J3');
        plot(handles.ax_scope1, t, th_deg(:, 4), 'm-', 'LineWidth', 1.2, 'DisplayName', 'J4');
        title(handles.ax_scope1, 'Joint Positions \theta(t)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope1, 'Angle (°)', 'FontSize', 7);
        legend(handles.ax_scope1, 'Location', 'northeast', 'FontSize', 6);

        % Subplot 2: Joint Velocities with Threshold lines
        cla(handles.ax_scope2);
        grid(handles.ax_scope2, 'on');
        hold(handles.ax_scope2, 'on');
        plot(handles.ax_scope2, t, dth_deg(:, 1), 'b-', 'LineWidth', 1.2);
        plot(handles.ax_scope2, t, dth_deg(:, 2), 'Color', [0.85, 0.35, 0.1], 'LineWidth', 1.2);
        plot(handles.ax_scope2, t, dth_deg(:, 3), 'Color', [0.15, 0.70, 0.25], 'LineWidth', 1.2);
        plot(handles.ax_scope2, t, dth_deg(:, 4), 'm-', 'LineWidth', 1.2);
        yline(handles.ax_scope2, 120.0, 'r--', 'LineWidth', 1.2);
        yline(handles.ax_scope2, -120.0, 'r--', 'LineWidth', 1.2);
        title(handles.ax_scope2, 'Joint Velocities \omega(t) & Limits (±120 °/s)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope2, 'Vel (°/s)', 'FontSize', 7);
        ylim(handles.ax_scope2, [-150, 150]);

        % Subplot 3: Joint Accelerations
        cla(handles.ax_scope3);
        grid(handles.ax_scope3, 'on');
        hold(handles.ax_scope3, 'on');
        plot(handles.ax_scope3, t, ddth_deg(:, 1), 'b-', 'LineWidth', 1.2);
        plot(handles.ax_scope3, t, ddth_deg(:, 2), 'Color', [0.85, 0.35, 0.1], 'LineWidth', 1.2);
        plot(handles.ax_scope3, t, ddth_deg(:, 3), 'Color', [0.15, 0.70, 0.25], 'LineWidth', 1.2);
        plot(handles.ax_scope3, t, ddth_deg(:, 4), 'm-', 'LineWidth', 1.2);
        title(handles.ax_scope3, 'Joint Accelerations \alpha(t)', 'FontSize', 8, 'FontWeight', 'bold');
        ylabel(handles.ax_scope3, 'Accel (°/s^2)', 'FontSize', 7);

        % Subplot 4: Tracking Error Norm
        cla(handles.ax_scope4);
        grid(handles.ax_scope4, 'on');
        hold(handles.ax_scope4, 'on');
        plot(handles.ax_scope4, t, err_norm, 'k-', 'LineWidth', 1.2);
        yline(handles.ax_scope4, 1.0, 'r:', 'LineWidth', 1.0, 'DisplayName', '1.0 mm Threshold');
        title(handles.ax_scope4, 'Cartesian Tracking Error Norm ||P_{act} - P_{des}||', 'FontSize', 8, 'FontWeight', 'bold');
        xlabel(handles.ax_scope4, 'Time (s)', 'FontSize', 7);
        ylabel(handles.ax_scope4, 'Error (mm)', 'FontSize', 7);
    end

    %% Export Motion Metrics Summary
    function metrics = export_motion_metrics()
        metrics = struct();
        if isempty(state.traj_struct)
            metrics.total_time = 0.0;
            metrics.max_velocity_deg = 0.0;
            metrics.max_accel_deg = 0.0;
            metrics.max_tracking_error_mm = 0.0;
            metrics.rms_tracking_error_mm = 0.0;
            metrics.in_bounds = true;
            return;
        end

        traj = state.traj_struct;
        metrics.total_time = traj.t(end) - traj.t(1);
        metrics.max_velocity_deg = rad2deg(max(abs(traj.dtheta(:))));
        metrics.max_accel_deg = rad2deg(max(abs(traj.ddtheta(:))));
        metrics.in_bounds = (metrics.max_velocity_deg <= 120.001);

        if isfield(traj, 'xyz_actual') && ~isempty(traj.xyz_actual)
            errs = zeros(numel(traj.t), 1);
            for i = 1:numel(traj.t)
                [p_act, ~] = forward_kinematics(traj.theta(i, :), 'rad');
                errs(i) = norm(p_act(1:3) - traj.xyz_actual(i, :));
            end
            metrics.max_tracking_error_mm = max(errs);
            metrics.rms_tracking_error_mm = sqrt(mean(errs.^2));
        else
            metrics.max_tracking_error_mm = 0.0;
            metrics.rms_tracking_error_mm = 0.0;
        end
    end

    %% Fallback trajectory planner if multi_point_planner not loaded
    function traj = generate_fallback_trajectory(cart_wp, durations, dt)
        n_seg = size(cart_wp, 1) - 1;
        q_wp = zeros(n_seg + 1, 4);
        for i = 1:(n_seg + 1)
            [th, ~] = inverse_kinematics(cart_wp(i, 1), cart_wp(i, 2), cart_wp(i, 3), cart_wp(i, 4));
            q_wp(i, :) = th;
        end

        all_t = [];
        all_th = [];
        all_dth = [];
        all_ddth = [];
        cur_t = 0.0;

        for seg = 1:n_seg
            t_seg = linspace(0, durations(seg), ceil(durations(seg)/dt) + 1);
            if seg > 1, t_seg(1) = []; end
            s_mat = zeros(numel(t_seg), 4);
            ds_mat = zeros(numel(t_seg), 4);
            dds_mat = zeros(numel(t_seg), 4);
            for j = 1:4
                % Standard quintic
                tau = t_seg / durations(seg);
                poly_pos = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
                poly_vel = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / durations(seg);
                poly_acc = (60*tau - 180*tau.^2 + 120*tau.^3) / (durations(seg)^2);
                s_mat(:, j) = q_wp(seg, j) + (q_wp(seg+1, j) - q_wp(seg, j)) * poly_pos(:);
                ds_mat(:, j) = (q_wp(seg+1, j) - q_wp(seg, j)) * poly_vel(:);
                dds_mat(:, j) = (q_wp(seg+1, j) - q_wp(seg, j)) * poly_acc(:);
            end
            all_t = [all_t; cur_t + t_seg(:)];
            all_th = [all_th; s_mat];
            all_dth = [all_dth; ds_mat];
            all_ddth = [all_ddth; dds_mat];
            cur_t = all_t(end);
        end

        traj.t = all_t;
        traj.theta = all_th;
        traj.dtheta = all_dth;
        traj.ddtheta = all_ddth;
        traj.xyz_actual = zeros(numel(all_t), 3);
        for i = 1:numel(all_t)
            [p, ~] = forward_kinematics(all_th(i, :), 'rad');
            traj.xyz_actual(i, :) = p(1:3);
        end
    end

    %% Clean resource shutdown
    function cleanup_resources()
        stop_animation_timer();
    end

    function on_close_fig(src, ~)
        cleanup_resources();
        delete(src);
    end
end
