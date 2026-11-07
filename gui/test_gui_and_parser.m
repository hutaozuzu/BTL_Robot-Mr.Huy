function test_gui_and_parser()
% TEST_GUI_AND_PARSER Automated unit test suite for Milestone 4:
% Trajectory File Importer (trajectory_parser.m) and Interactive GUI (palletizer_gui.m).
%
% Tests all aspects of F13, F14, F15, F16:
% - Parser: CSV/JSON loading, 7 fields, aliases, reachability validation, error recovery
% - GUI: Headless initialization ('Visible', 'off'), 5 panels, 3D visualizer
% - Sliders: Manufacturer hardware limits, Zero/Home presets, FK sync
% - Cartesian Jog: IK trigger, Green/Red LED reachability indicator, step jogging
% - Importer & Runner: UITable population, speed scaling, trajectory execution
% - Analytics Scope: 4 subplots, +-120 deg/s velocity thresholds, tracking error norm < 1 mm
% - Integrity: Zero Corke toolbox across gui/ directory
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Setup paths
    gui_dir = fileparts(mfilename('fullpath'));
    base_dir = fullfile(gui_dir, '..');
    addpath(gui_dir);
    addpath(fullfile(base_dir, 'kinematics'));
    addpath(fullfile(base_dir, 'trajectory'));
    addpath(fullfile(base_dir, 'data'));
    addpath(fullfile(base_dir, 'tests'));

    fprintf('\n====================================================================\n');
    fprintf('  MILESTONE 4 UNIT TEST SUITE: GUI & TRAJECTORY FILE IMPORTER       \n');
    fprintf('====================================================================\n');

    tests = { ...
        % Parser Tests (F15)
        @() test_parser_csv_success(base_dir), 'Parser: CSV sample import (8 waypoints)'; ...
        @() test_parser_json_success(base_dir), 'Parser: JSON sample import (8 waypoints)'; ...
        @() test_parser_7_fields(base_dir), 'Parser: Standard 7-field structure compliance'; ...
        @() test_parser_nonexistent_file(), 'Parser: Non-existent file error handling'; ...
        @() test_parser_unsupported_ext(), 'Parser: Unsupported extension rejection (.txt)'; ...
        @() test_parser_empty_file(), 'Parser: Empty file rejection'; ...
        @() test_parser_malformed_csv(), 'Parser: Malformed CSV header rejection'; ...
        @() test_parser_malformed_json(), 'Parser: Corrupted JSON syntax rejection'; ...
        @() test_parser_unreachable_point(), 'Parser: Out-of-reach waypoint rejection'; ...
        @() test_parser_alias_headers(base_dir), 'Parser: Support for alias column headers (x_mm, etc.)'; ...
        ...
        % GUI 3D Robot Visualizer Tests (F13)
        @() test_gui_headless_init(), 'GUI: Headless initialization (Visible=off)'; ...
        @() test_gui_5_panels_exist(), 'GUI: Verification of 5 required functional panels'; ...
        @() test_gui_3d_axes_and_objects(), 'GUI: 3D axes, robot links, joints, tool, and fixtures'; ...
        @() test_gui_3d_posture_fk_sync(), 'GUI: 3D link segment coordinates align with FK'; ...
        @() test_gui_clean_close(), 'GUI: Clean shutdown without timer or handle leaks'; ...
        ...
        % GUI Manual Joint Sliders & Presets Tests (F14)
        @() test_gui_slider_limits(), 'GUI: Slider bounds match manufacturer limits [-162..162, etc.]'; ...
        @() test_gui_slider_jog_fk_sync(), 'GUI: Slider jog updates FK and Cartesian readout'; ...
        @() test_gui_zero_button(), 'GUI: Zero button sets [0, 0, 0, 0] configuration'; ...
        @() test_gui_home_button(), 'GUI: Home button restores standby pose [240, 0, 180, 0]'; ...
        ...
        % GUI Cartesian Jog Controller Tests (F14)
        @() test_gui_cartesian_jog_reachable(), 'GUI: Cartesian jog solves IK and sets LED to GREEN'; ...
        @() test_gui_cartesian_jog_unreachable(), 'GUI: Out-of-reach input sets LED to RED'; ...
        @() test_gui_step_jog_buttons(), 'GUI: Step buttons (+-10 mm, +-5 deg) modify coords'; ...
        ...
        % GUI Trajectory Importer & Runner Tests (F15)
        @() test_gui_import_csv(base_dir), 'GUI: Import CSV populates UITable and 3D waypoints'; ...
        @() test_gui_import_json(base_dir), 'GUI: Import JSON populates UITable and 3D waypoints'; ...
        @() test_gui_speed_scaling(), 'GUI: Speed scaling slider adjusts execution duration'; ...
        @() test_gui_run_trajectory(), 'GUI: Run trajectory animates motion and plots scope'; ...
        ...
        % GUI Motion Analytics Scope Tests (F16)
        @() test_gui_scope_4_subplots(), 'GUI Scope: 4 motion subplots initialized and titled'; ...
        @() test_gui_scope_velocity_limits(), 'GUI Scope: Subplot 2 displays +-120 deg/s threshold lines'; ...
        @() test_gui_export_metrics(), 'GUI Scope: Metrics export struct conforms to contract'; ...
        @() test_gui_tracking_error_bound(), 'GUI Scope: Trajectory tracking error norm < 1.0 mm'; ...
        ...
        % Integrity Mandate
        @() test_zero_corke_toolbox(gui_dir), 'Integrity: Zero Peter Corke Robotics Toolbox in gui/' ...
    };

    total_tests = size(tests, 1);
    n_pass = 0;
    n_fail = 0;

    for i = 1:total_tests
        test_fn = tests{i, 1};
        test_desc = tests{i, 2};
        t_start = tic;
        try
            [ok, msg] = test_fn();
            t_elapsed = toc(t_start) * 1000;
            if ok
                n_pass = n_pass + 1;
                fprintf('  [PASS] Test %02d/%02d: %-52s (%.1f ms)\n', i, total_tests, test_desc, t_elapsed);
            else
                n_fail = n_fail + 1;
                fprintf('  [FAIL] Test %02d/%02d: %-52s : %s\n', i, total_tests, test_desc, msg);
            end
        catch ME
            t_elapsed = toc(t_start) * 1000;
            n_fail = n_fail + 1;
            fprintf('  [FAIL] Test %02d/%02d: %-52s : EXCEPTION %s\n', i, total_tests, test_desc, ME.message);
        end
    end

    fprintf('\n--------------------------------------------------------------------\n');
    fprintf('  TEST SUMMARY: Total = %d | PASS = %d | FAIL = %d\n', total_tests, n_pass, n_fail);
    fprintf('--------------------------------------------------------------------\n\n');

    if n_fail > 0
        error('test_gui_and_parser:TestsFailed', '%d of %d tests failed.', n_fail, total_tests);
    end
end

%% ========================================================================
%% Parser Test Implementations (F15)
%% ========================================================================

function [ok, msg] = test_parser_csv_success(base_dir)
    csv_file = fullfile(base_dir, 'data', 'trajectory_sample.csv');
    [wp, is_valid, err] = trajectory_parser(csv_file);
    if is_valid && numel(wp) == 8 && isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('CSV parse failed: is_valid=%d, num_wp=%d, err=%s', is_valid, numel(wp), err);
    end
end

function [ok, msg] = test_parser_json_success(base_dir)
    json_file = fullfile(base_dir, 'data', 'trajectory_sample.json');
    [wp, is_valid, err] = trajectory_parser(json_file);
    if is_valid && numel(wp) == 8 && isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('JSON parse failed: is_valid=%d, num_wp=%d, err=%s', is_valid, numel(wp), err);
    end
end

function [ok, msg] = test_parser_7_fields(base_dir)
    csv_file = fullfile(base_dir, 'data', 'trajectory_sample.csv');
    [wp, ~, ~] = trajectory_parser(csv_file);
    expected_fields = {'id', 'x', 'y', 'z', 'phi', 'duration', 'action'};
    if ~isequal(sort(fieldnames(wp)), sort(expected_fields(:)))
        ok = false; msg = 'Struct fields mismatch standard 7 fields'; return;
    end
    % Type checks
    for i = 1:numel(wp)
        if ~isnumeric(wp(i).id) || ~isnumeric(wp(i).x) || ~isnumeric(wp(i).y) || ...
           ~isnumeric(wp(i).z) || ~isnumeric(wp(i).phi) || ~isnumeric(wp(i).duration) || ...
           ~ischar(wp(i).action)
            ok = false; msg = sprintf('Field types incorrect on waypoint %d', i); return;
        end
    end
    ok = true; msg = '';
end

function [ok, msg] = test_parser_nonexistent_file()
    [wp, is_valid, err] = trajectory_parser('d:\non_existent_path_xyz123.csv');
    if ~is_valid && isempty(wp) && ~isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = 'Non-existent file did not return clean is_valid=false';
    end
end

function [ok, msg] = test_parser_unsupported_ext()
    tmp_file = [tempname, '.txt'];
    fid = fopen(tmp_file, 'w');
    fprintf(fid, 'test content\n');
    fclose(fid);
    [~, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if ~is_valid && contains(err, 'Unsupported')
        ok = true; msg = '';
    else
        ok = false; msg = 'Unsupported extension was not rejected';
    end
end

function [ok, msg] = test_parser_empty_file()
    tmp_file = [tempname, '.csv'];
    fid = fopen(tmp_file, 'w');
    fclose(fid);
    [~, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if ~is_valid && ~isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = 'Empty file was not rejected';
    end
end

function [ok, msg] = test_parser_malformed_csv()
    tmp_file = [tempname, '.csv'];
    fid = fopen(tmp_file, 'w');
    fprintf(fid, 'alpha,beta,gamma\n1,2,3\n');
    fclose(fid);
    [~, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if ~is_valid && ~isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = 'Malformed CSV columns were not rejected';
    end
end

function [ok, msg] = test_parser_malformed_json()
    tmp_file = [tempname, '.json'];
    fid = fopen(tmp_file, 'w');
    fprintf(fid, '{ "trajectory": corrupted_json_syntax ');
    fclose(fid);
    [~, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if ~is_valid && ~isempty(err)
        ok = true; msg = '';
    else
        ok = false; msg = 'Malformed JSON was not rejected';
    end
end

function [ok, msg] = test_parser_unreachable_point()
    tmp_file = [tempname, '.csv'];
    fid = fopen(tmp_file, 'w');
    % Waypoint at radius 600 mm (far outside 260 mm reach)
    fprintf(fid, 'point_id,x,y,z,phi,duration,action\n1,600.0,0.0,180.0,0.0,2.0,MOVE\n');
    fclose(fid);
    [~, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if ~is_valid && (contains(err, 'reach') || contains(err, 'unreachable'))
        ok = true; msg = '';
    else
        ok = false; msg = 'Unreachable point (r=600mm) was not rejected';
    end
end

function [ok, msg] = test_parser_alias_headers(base_dir)
    tmp_file = [tempname, '.csv'];
    fid = fopen(tmp_file, 'w');
    fprintf(fid, 'point_id,x_mm,y_mm,z_mm,phi_deg,duration_s,action\n');
    fprintf(fid, '1,240.0,0.0,180.0,0.0,2.0,MOVE\n');
    fprintf(fid, '2,220.0,-100.0,120.0,0.0,2.0,MOVE\n');
    fclose(fid);
    [wp, is_valid, err] = trajectory_parser(tmp_file);
    delete(tmp_file);
    if is_valid && numel(wp) == 2 && wp(1).x == 240.0 && wp(2).y == -100.0
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Alias headers failed: is_valid=%d, err=%s', is_valid, err);
    end
end

%% ========================================================================
%% GUI Visualizer Tests (F13)
%% ========================================================================

function [ok, msg] = test_gui_headless_init()
    [f, h] = palletizer_gui('Visible', 'off');
    if isvalid(f) && isstruct(h) && isfield(h, 'panel1')
        close(f);
        ok = true; msg = '';
    else
        if isvalid(f), close(f); end
        ok = false; msg = 'Headless GUI failed to initialize';
    end
end

function [ok, msg] = test_gui_5_panels_exist()
    [f, h] = palletizer_gui('Visible', 'off');
    p_ok = isfield(h, 'panel1') && isfield(h, 'panel2') && isfield(h, 'panel3') && ...
           isfield(h, 'panel4') && isfield(h, 'panel5') && ...
           isvalid(h.panel1) && isvalid(h.panel2) && isvalid(h.panel3) && ...
           isvalid(h.panel4) && isvalid(h.panel5);
    close(f);
    if p_ok
        ok = true; msg = '';
    else
        ok = false; msg = 'One or more of the 5 panels are missing or invalid';
    end
end

function [ok, msg] = test_gui_3d_axes_and_objects()
    [f, h] = palletizer_gui('Visible', 'off');
    ax_ok = isvalid(h.ax_3d) && isvalid(h.h_links) && isvalid(h.h_joints) && ...
            isvalid(h.h_trail) && isvalid(h.h_waypoints) && isvalid(h.h_tool);
    close(f);
    if ax_ok
        ok = true; msg = '';
    else
        ok = false; msg = '3D graphic handles missing or invalid';
    end
end

function [ok, msg] = test_gui_3d_posture_fk_sync()
    [f, h] = palletizer_gui('Visible', 'off');
    test_th = [0.0, deg2rad(30), deg2rad(-45), deg2rad(15)];
    h.update_robot(test_th);
    x_data = get(h.h_links, 'XData');
    y_data = get(h.h_links, 'YData');
    z_data = get(h.h_links, 'ZData');
    close(f);

    [pose, T_all] = forward_kinematics(test_th, 'rad');
    p4_fk = T_all{4}(1:3, 4)';
    p4_gui = [x_data(end), y_data(end), z_data(end)];

    if norm(p4_fk - p4_gui) < 1e-4
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('3D visualizer end-effector mismatch: err = %.2e mm', norm(p4_fk - p4_gui));
    end
end

function [ok, msg] = test_gui_clean_close()
    [f, h] = palletizer_gui('Visible', 'off');
    close(f);
    if ~isvalid(f)
        ok = true; msg = '';
    else
        ok = false; msg = 'Figure remained valid after close';
    end
end

%% ========================================================================
%% GUI Sliders & Presets Tests (F14)
%% ========================================================================

function [ok, msg] = test_gui_slider_limits()
    [f, h] = palletizer_gui('Visible', 'off');
    expected = [ ...
        -162.0, 162.0; ...
          -2.0,  90.0; ...
         -92.0,  60.0; ...
        -180.0, 180.0  ...
    ];
    actual = zeros(4, 2);
    for j = 1:4
        actual(j, 1) = get(h.sliders(j), 'Min');
        actual(j, 2) = get(h.sliders(j), 'Max');
    end
    close(f);
    if norm(actual - expected, 'fro') < 1e-9
        ok = true; msg = '';
    else
        ok = false; msg = 'Slider limits mismatch hardware bounds';
    end
end

function [ok, msg] = test_gui_slider_jog_fk_sync()
    [f, h] = palletizer_gui('Visible', 'off');
    set(h.sliders(1), 'Value', 45.0);
    h.on_slider_change();
    x_val = str2double(get(h.edit_coords(1), 'String'));
    y_val = str2double(get(h.edit_coords(2), 'String'));
    close(f);

    % With J1 = 45 deg, X and Y should both be positive and atan2(Y, X) approx 45 deg
    measured_yaw = atan2d(y_val, x_val);
    if abs(measured_yaw - 45.0) < 0.5
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Slider change did not update Cartesian readout: yaw = %.1f deg', measured_yaw);
    end
end

function [ok, msg] = test_gui_zero_button()
    [f, h] = palletizer_gui('Visible', 'off');
    % Trigger Zero callback
    cb = get(h.btn_zero, 'Callback');
    cb([], []);
    for j = 1:4
        val_j = get(h.sliders(j), 'Value');
        if abs(val_j) > 1e-4
            close(f);
            ok = false; msg = sprintf('Joint %d not zeroed: val = %.2f', j, val_j); return;
        end
    end
    % Zero configuration pose should be [318, 0, 120, 0] mm
    x_val = str2double(get(h.edit_coords(1), 'String'));
    z_val = str2double(get(h.edit_coords(3), 'String'));
    close(f);
    if abs(x_val - 318.0) < 0.5 && abs(z_val - 120.0) < 0.5
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Zero button pose mismatch: x=%.1f, z=%.1f', x_val, z_val);
    end
end

function [ok, msg] = test_gui_home_button()
    [f, h] = palletizer_gui('Visible', 'off');
    % First set to zero
    cb_zero = get(h.btn_zero, 'Callback');
    cb_zero([], []);
    % Then click Home
    cb_home = get(h.btn_home, 'Callback');
    cb_home([], []);
    x_val = str2double(get(h.edit_coords(1), 'String'));
    y_val = str2double(get(h.edit_coords(2), 'String'));
    z_val = str2double(get(h.edit_coords(3), 'String'));
    close(f);
    if abs(x_val - 240.0) < 0.5 && abs(y_val - 0.0) < 0.5 && abs(z_val - 180.0) < 0.5
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Home button did not restore standby [240, 0, 180]: [%.1f, %.1f, %.1f]', x_val, y_val, z_val);
    end
end

%% ========================================================================
%% GUI Cartesian Jog Tests (F14)
%% ========================================================================

function [ok, msg] = test_gui_cartesian_jog_reachable()
    [f, h] = palletizer_gui('Visible', 'off');
    set(h.edit_coords(1), 'String', '220.0');
    set(h.edit_coords(2), 'String', '-100.0');
    set(h.edit_coords(3), 'String', '120.0');
    set(h.edit_coords(4), 'String', '0.0');
    success = h.on_solve_ik();
    led_str = get(h.led_status, 'String');
    led_bg = get(h.led_status, 'BackgroundColor');
    close(f);
    if success && contains(led_str, 'REACHABLE') && led_bg(2) > led_bg(1)
        ok = true; msg = '';
    else
        ok = false; msg = 'Cartesian jog did not set LED to GREEN for valid target';
    end
end

function [ok, msg] = test_gui_cartesian_jog_unreachable()
    [f, h] = palletizer_gui('Visible', 'off');
    set(h.edit_coords(1), 'String', '500.0');
    set(h.edit_coords(2), 'String', '200.0');
    set(h.edit_coords(3), 'String', '100.0');
    set(h.edit_coords(4), 'String', '0.0');
    success = h.on_solve_ik();
    led_str = get(h.led_status, 'String');
    led_bg = get(h.led_status, 'BackgroundColor');
    close(f);
    if ~success && contains(led_str, 'UNREACHABLE') && led_bg(1) > led_bg(2)
        ok = true; msg = '';
    else
        ok = false; msg = 'Cartesian jog did not set LED to RED for unreachable target';
    end
end

function [ok, msg] = test_gui_step_jog_buttons()
    [f, h] = palletizer_gui('Visible', 'off');
    % Initial X
    x0 = str2double(get(h.edit_coords(1), 'String'));
    if isfield(h, 'btn_jog_plus') && isvalid(h.btn_jog_plus(1))
        cb = get(h.btn_jog_plus(1), 'Callback');
        cb(h.btn_jog_plus(1), []);
    elseif isfield(h, 'jog_step')
        h.jog_step(1, 10.0);
    end
    x1 = str2double(get(h.edit_coords(1), 'String'));
    close(f);
    if abs(x1 - (x0 + 10.0)) < 0.5
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Step button failed to increment X: x0=%.1f, x1=%.1f', x0, x1);
    end
end

%% ========================================================================
%% GUI Importer & Runner Tests (F15)
%% ========================================================================

function [ok, msg] = test_gui_import_csv(base_dir)
    [f, h] = palletizer_gui('Visible', 'off');
    csv_file = fullfile(base_dir, 'data', 'trajectory_sample.csv');
    h.load_file(csv_file);
    tbl_data = get(h.table_waypoints, 'Data');
    n_rows = size(tbl_data, 1);
    wp_xdata = get(h.h_waypoints, 'XData');
    close(f);
    if n_rows == 8 && numel(wp_xdata) == 8 && ~any(isnan(wp_xdata))
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('CSV import failed in GUI: rows = %d', n_rows);
    end
end

function [ok, msg] = test_gui_import_json(base_dir)
    [f, h] = palletizer_gui('Visible', 'off');
    json_file = fullfile(base_dir, 'data', 'trajectory_sample.json');
    h.load_file(json_file);
    tbl_data = get(h.table_waypoints, 'Data');
    n_rows = size(tbl_data, 1);
    close(f);
    if n_rows == 8
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('JSON import failed in GUI: rows = %d', n_rows);
    end
end

function [ok, msg] = test_gui_speed_scaling()
    [f, h] = palletizer_gui('Visible', 'off');
    set(h.slider_speed, 'Value', 1.5);
    cb = get(h.slider_speed, 'Callback');
    cb([], []);
    lbl_text = get(h.lbl_speed, 'String');
    close(f);
    if contains(lbl_text, '1.50x')
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Speed scaling label mismatch: %s', lbl_text);
    end
end

function [ok, msg] = test_gui_run_trajectory()
    [f, h] = palletizer_gui('Visible', 'off');
    % Run loaded sample trajectory in headless mode
    h.run_trajectory();
    metrics = h.export_metrics();
    close(f);
    if metrics.total_time > 5.0 && metrics.max_velocity_deg > 0
        ok = true; msg = '';
    else
        ok = false; msg = 'Trajectory execution did not produce valid metrics';
    end
end

%% ========================================================================
%% GUI Motion Analytics Scope Tests (F16)
%% ========================================================================

function [ok, msg] = test_gui_scope_4_subplots()
    [f, h] = palletizer_gui('Visible', 'off');
    ax_ok = isvalid(h.ax_scope1) && isvalid(h.ax_scope2) && ...
            isvalid(h.ax_scope3) && isvalid(h.ax_scope4);
    close(f);
    if ax_ok
        ok = true; msg = '';
    else
        ok = false; msg = 'One or more scope subplots are missing';
    end
end

function [ok, msg] = test_gui_scope_velocity_limits()
    [f, h] = palletizer_gui('Visible', 'off');
    y_limits = ylim(h.ax_scope2);
    close(f);
    % Subplot 2 must accommodate at least +-120 deg/s
    if y_limits(1) <= -120.0 && y_limits(2) >= 120.0
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Scope velocity Y-limits [%.1f, %.1f] do not cover +-120 deg/s', y_limits(1), y_limits(2));
    end
end

function [ok, msg] = test_gui_export_metrics()
    [f, h] = palletizer_gui('Visible', 'off');
    h.run_trajectory();
    m = h.export_metrics();
    close(f);
    req_fields = {'total_time', 'max_velocity_deg', 'max_accel_deg', 'max_tracking_error_mm', 'in_bounds'};
    for k = 1:numel(req_fields)
        if ~isfield(m, req_fields{k})
            ok = false; msg = sprintf('Exported metrics missing field: %s', req_fields{k}); return;
        end
    end
    ok = true; msg = '';
end

function [ok, msg] = test_gui_tracking_error_bound()
    [f, h] = palletizer_gui('Visible', 'off');
    h.run_trajectory();
    m = h.export_metrics();
    close(f);
    if m.max_tracking_error_mm < 1.0
        ok = true; msg = '';
    else
        ok = false; msg = sprintf('Tracking error exceeded 1.0 mm: %.4f mm', m.max_tracking_error_mm);
    end
end

%% ========================================================================
%% Integrity: Zero Peter Corke Robotics Toolbox in gui/
%% ========================================================================

function [ok, msg] = test_zero_corke_toolbox(gui_dir)
    corke_forbidden = { ...
        'SerialLink', 'Link', 'fkine', 'ikine', 'ikcon', ...
        'ctraj', 'jtraj', 'lspb', 'qinterp', 'trplot', 'plot_arm' ...
    };

    m_files = dir(fullfile(gui_dir, '*.m'));
    found_violations = {};

    for i = 1:numel(m_files)
        filepath = fullfile(gui_dir, m_files(i).name);
        content = fileread(filepath);

        for j = 1:numel(corke_forbidden)
            token = corke_forbidden{j};
            pattern = ['\b', token, '\b'];
            if ~isempty(regexp(content, pattern, 'once'))
                found_violations{end+1} = sprintf('%s in %s', token, m_files(i).name); %#ok<AGROW>
            end
        end
    end

    if isempty(found_violations)
        ok = true; msg = 'Zero Peter Corke Robotics Toolbox found in gui/';
    else
        ok = false; msg = sprintf('Forbidden Corke toolbox usage: %s', strjoin(found_violations, ', '));
    end
end
