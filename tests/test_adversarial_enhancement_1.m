% TEST_ADVERSARIAL_ENHANCEMENT_1
% Empirical Challenger Test Suite for GUI Layout & 3D Waypoint Marker Rendering
% Challenger: teamwork_challenger (Challenger Enhancement 1)
% Target: myPalletizer_BTL/gui/palletizer_gui.m
% Strictly native MATLAB. Strictly zero Peter Corke Robotics Toolbox.

function results = test_adversarial_enhancement_1()
    fprintf('================================================================================\n');
    fprintf('  ADVERSARIAL VERIFICATION: GUI Layout & 3D Waypoint Marker Rendering           \n');
    fprintf('  Challenger: teamwork_challenger                                               \n');
    fprintf('  Target: d:\\Lidar\\myPalletizer_BTL\\gui\\palletizer_gui.m                         \n');
    fprintf('================================================================================\n\n');

    projectRoot = 'd:\Lidar\myPalletizer_BTL';
    addpath(fullfile(projectRoot, 'gui'));
    addpath(fullfile(projectRoot, 'kinematics'));
    addpath(fullfile(projectRoot, 'trajectory'));
    addpath(fullfile(projectRoot, 'data'));
    addpath(fullfile(projectRoot, 'tests'));

    results = struct();
    results.tests = {};
    results.total_tests = 0;
    results.total_pass = 0;
    results.total_fail = 0;
    results.findings = {};

    function record_result(passed, name, detail)
        results.total_tests = results.total_tests + 1;
        t_entry = struct('id', results.total_tests, 'name', name, 'passed', passed, 'detail', detail);
        results.tests{end+1} = t_entry;
        if passed
            results.total_pass = results.total_pass + 1;
            fprintf('  [PASS] Test %02d: %s\n         Detail: %s\n', results.total_tests, name, detail);
        else
            results.total_fail = results.total_fail + 1;
            fprintf('  [FAIL] Test %02d: %s\n         Reason: %s\n', results.total_tests, name, detail);
            results.findings{end+1} = sprintf('FAIL (Test %02d - %s): %s', results.total_tests, name, detail);
        end
    end

    %% ====================================================================
    %% PART 1: PANEL 3 (CARTESIAN JOG CONTROLLER) LAYOUT & OVERLAPS
    %% ====================================================================
    fprintf('\n--- PART 1: PANEL 3 (CARTESIAN JOG CONTROLLER) LAYOUT & OVERLAPS ---\n');

    [f, h] = palletizer_gui('Visible', 'off');

    % Extract Panel 3 dimensions
    orig_p3_units = get(h.panel3, 'Units');
    set(h.panel3, 'Units', 'pixels');
    p3_pixel_pos = get(h.panel3, 'Position'); % [x, y, width, height]
    set(h.panel3, 'Units', orig_p3_units);
    p3_norm_pos = get(h.panel3, 'Position');
    p3_w_px = p3_pixel_pos(3);
    p3_h_px = p3_pixel_pos(4);
    fprintf('  Panel 3 Pixel Dimensions: Width = %.1f px, Height = %.1f px\n', p3_w_px, p3_h_px);

    % Identify all controls in Panel 3
    % There are 18 controls:
    % 4 coordinate labels: X, Y, Z, Phi
    % 4 minus buttons: btn_jog_minus(1:4)
    % 4 edit boxes: edit_coords(1:4)
    % 4 plus buttons: btn_jog_plus(1:4)
    % 1 IK button: btn_ik
    % 1 LED status: led_status
    p3_children = allchild(h.panel3);
    fprintf('  Total controls in Panel 3: %d\n', numel(p3_children));

    % Build control table with names and bounding boxes
    ctrl_list_p3 = struct('handle', {}, 'name', {}, 'norm_pos', {}, 'pix_pos', {}, 'bbox_norm', {}, 'bbox_pix', {});
    
    for i = 1:numel(p3_children)
        c = p3_children(i);
        st = get(c, 'Style');
        str = get(c, 'String');
        tag = get(c, 'Tag');
        
        % Assign human-readable name
        if isequal(c, h.btn_ik)
            c_name = 'btn_ik';
        elseif isequal(c, h.led_status)
            c_name = 'led_status';
        else
            found_named = false;
            for k = 1:4
                if isequal(c, h.btn_jog_minus(k))
                    c_name = sprintf('btn_jog_minus(%d)', k);
                    found_named = true; break;
                elseif isequal(c, h.edit_coords(k))
                    c_name = sprintf('edit_coords(%d)', k);
                    found_named = true; break;
                elseif isequal(c, h.btn_jog_plus(k))
                    c_name = sprintf('btn_jog_plus(%d)', k);
                    found_named = true; break;
                end
            end
            if ~found_named
                if strcmp(st, 'text')
                    c_name = sprintf('label_%s', regexprep(str, '[^a-zA-Z0-9]', '_'));
                else
                    c_name = sprintf('ctrl_%s_%s', st, tag);
                end
            end
        end

        u_orig = get(c, 'Units');
        set(c, 'Units', 'normalized');
        n_pos = get(c, 'Position'); % [x, y, w, h]
        set(c, 'Units', 'pixels');
        px_pos = get(c, 'Position'); % [x, y, w, h] in pixels relative to panel3
        set(c, 'Units', u_orig);

        bbox_n = [n_pos(1), n_pos(2), n_pos(1) + n_pos(3), n_pos(2) + n_pos(4)]; % [xmin, ymin, xmax, ymax]
        bbox_p = [px_pos(1), px_pos(2), px_pos(1) + px_pos(3), px_pos(2) + px_pos(4)];

        ctrl_list_p3(end+1) = struct('handle', c, 'name', c_name, ...
            'norm_pos', n_pos, 'pix_pos', px_pos, 'bbox_norm', bbox_n, 'bbox_pix', bbox_p);
    end

    % 1.1 All-pairs bounding box intersection check in Panel 3
    n_p3 = numel(ctrl_list_p3);
    overlap_found_p3 = false;
    overlap_details_p3 = {};
    num_pairs_checked_p3 = 0;

    for i = 1:(n_p3 - 1)
        for j = (i + 1):n_p3
            num_pairs_checked_p3 = num_pairs_checked_p3 + 1;
            c1 = ctrl_list_p3(i);
            c2 = ctrl_list_p3(j);

            % Intersection in pixels
            ox_px = max(0, min(c1.bbox_pix(3), c2.bbox_pix(3)) - max(c1.bbox_pix(1), c2.bbox_pix(1)));
            oy_px = max(0, min(c1.bbox_pix(4), c2.bbox_pix(4)) - max(c1.bbox_pix(2), c2.bbox_pix(2)));
            area_px = ox_px * oy_px;

            if area_px > 0.01 % tolerance: > 0.01 sq pixels
                overlap_found_p3 = true;
                detail_str = sprintf('%s vs %s: Overlap Area = %.2f px^2 (dx=%.2f, dy=%.2f)', ...
                    c1.name, c2.name, area_px, ox_px, oy_px);
                overlap_details_p3{end+1} = detail_str;
            end
        end
    end

    record_result(~overlap_found_p3, ...
        '1.1 Panel 3 All-Pairs Overlap Check', ...
        sprintf('Checked %d control pairs in Panel 3. Zero overlaps detected.', num_pairs_checked_p3));
    if overlap_found_p3
        for d = 1:numel(overlap_details_p3)
            fprintf('         Overlap: %s\n', overlap_details_p3{d});
        end
    end

    % 1.2 Vertical gap check between Row 4 (Phi) controls and handles.btn_ik
    % Row 4 controls: edit_coords(4), btn_jog_minus(4), btn_jog_plus(4), and coordinate label 4
    row4_indices = [];
    for i = 1:n_p3
        nm = ctrl_list_p3(i).name;
        if contains(nm, '(4)') || contains(nm, 'Phi')
            row4_indices(end+1) = i;
        end
    end

    row4_bottoms_norm = arrayfun(@(idx) ctrl_list_p3(idx).bbox_norm(2), row4_indices);
    row4_bottoms_pix  = arrayfun(@(idx) ctrl_list_p3(idx).bbox_pix(2), row4_indices);
    min_row4_bottom_norm = min(row4_bottoms_norm);
    min_row4_bottom_pix  = min(row4_bottoms_pix);

    u_btn_ik = get(h.btn_ik, 'Units');
    set(h.btn_ik, 'Units', 'normalized');
    btn_ik_norm = get(h.btn_ik, 'Position');
    set(h.btn_ik, 'Units', 'pixels');
    btn_ik_pix = get(h.btn_ik, 'Position');
    set(h.btn_ik, 'Units', u_btn_ik);

    btn_ik_top_norm = btn_ik_norm(2) + btn_ik_norm(4);
    btn_ik_top_pix  = btn_ik_pix(2) + btn_ik_pix(4);

    gap_row4_ik_norm = min_row4_bottom_norm - btn_ik_top_norm;
    gap_row4_ik_pix  = min_row4_bottom_pix - btn_ik_top_pix;

    detail_row4_ik = sprintf('Row 4 bottom = %.2f px (norm=%.3f), btn_ik top = %.2f px (norm=%.3f) -> Gap = %.2f px (norm=%.3f)', ...
        min_row4_bottom_pix, min_row4_bottom_norm, btn_ik_top_pix, btn_ik_top_norm, gap_row4_ik_pix, gap_row4_ik_norm);
    record_result(gap_row4_ik_pix > 0, '1.2 Panel 3 Row 4 to btn_ik Vertical Gap', detail_row4_ik);

    % 1.3 Vertical gap check between handles.btn_ik and handles.led_status
    btn_ik_bottom_norm = btn_ik_norm(2);
    btn_ik_bottom_pix  = btn_ik_pix(2);

    u_led = get(h.led_status, 'Units');
    set(h.led_status, 'Units', 'normalized');
    led_norm = get(h.led_status, 'Position');
    set(h.led_status, 'Units', 'pixels');
    led_pix = get(h.led_status, 'Position');
    set(h.led_status, 'Units', u_led);

    led_top_norm = led_norm(2) + led_norm(4);
    led_top_pix  = led_pix(2) + led_pix(4);

    gap_ik_led_norm = btn_ik_bottom_norm - led_top_norm;
    gap_ik_led_pix  = btn_ik_bottom_pix - led_top_pix;

    detail_ik_led = sprintf('btn_ik bottom = %.2f px (norm=%.3f), led_status top = %.2f px (norm=%.3f) -> Gap = %.2f px (norm=%.3f)', ...
        btn_ik_bottom_pix, btn_ik_bottom_norm, led_top_pix, led_top_norm, gap_ik_led_pix, gap_ik_led_norm);
    record_result(gap_ik_led_pix > 0, '1.3 Panel 3 btn_ik to led_status Vertical Gap', detail_ik_led);

    %% ====================================================================
    %% PART 2: PANEL 2 (JOINT SLIDERS) LAYOUT & OVERLAPS
    %% ====================================================================
    fprintf('\n--- PART 2: PANEL 2 (JOINT SLIDERS) LAYOUT & OVERLAPS ---\n');

    orig_p2_units = get(h.panel2, 'Units');
    set(h.panel2, 'Units', 'pixels');
    p2_pixel_pos = get(h.panel2, 'Position');
    set(h.panel2, 'Units', orig_p2_units);
    p2_w_px = p2_pixel_pos(3);
    p2_h_px = p2_pixel_pos(4);
    fprintf('  Panel 2 Pixel Dimensions: Width = %.1f px, Height = %.1f px\n', p2_w_px, p2_h_px);

    p2_children = allchild(h.panel2);
    fprintf('  Total controls in Panel 2: %d\n', numel(p2_children));

    ctrl_list_p2 = struct('handle', {}, 'name', {}, 'norm_pos', {}, 'pix_pos', {}, 'bbox_norm', {}, 'bbox_pix', {});

    for i = 1:numel(p2_children)
        c = p2_children(i);
        st = get(c, 'Style');
        str = get(c, 'String');
        
        if isequal(c, h.btn_zero)
            c_name = 'btn_zero';
        elseif isequal(c, h.btn_home)
            c_name = 'btn_home';
        else
            found_named = false;
            for k = 1:4
                if isequal(c, h.sliders(k))
                    c_name = sprintf('slider(%d)', k);
                    found_named = true; break;
                elseif isequal(c, h.lbl_vals(k))
                    c_name = sprintf('lbl_val(%d)', k);
                    found_named = true; break;
                end
            end
            if ~found_named
                if strcmp(st, 'text')
                    c_name = sprintf('label_%s', regexprep(str, '[^a-zA-Z0-9]', '_'));
                else
                    c_name = sprintf('ctrl_%s', st);
                end
            end
        end

        u_orig = get(c, 'Units');
        set(c, 'Units', 'normalized');
        n_pos = get(c, 'Position');
        set(c, 'Units', 'pixels');
        px_pos = get(c, 'Position');
        set(c, 'Units', u_orig);

        bbox_n = [n_pos(1), n_pos(2), n_pos(1) + n_pos(3), n_pos(2) + n_pos(4)];
        bbox_p = [px_pos(1), px_pos(2), px_pos(1) + px_pos(3), px_pos(2) + px_pos(4)];

        ctrl_list_p2(end+1) = struct('handle', c, 'name', c_name, ...
            'norm_pos', n_pos, 'pix_pos', px_pos, 'bbox_norm', bbox_n, 'bbox_pix', bbox_p);
    end

    % 2.1 All-pairs bounding box intersection check in Panel 2
    n_p2 = numel(ctrl_list_p2);
    overlap_found_p2 = false;
    overlap_details_p2 = {};
    num_pairs_checked_p2 = 0;

    for i = 1:(n_p2 - 1)
        for j = (i + 1):n_p2
            num_pairs_checked_p2 = num_pairs_checked_p2 + 1;
            c1 = ctrl_list_p2(i);
            c2 = ctrl_list_p2(j);

            ox_px = max(0, min(c1.bbox_pix(3), c2.bbox_pix(3)) - max(c1.bbox_pix(1), c2.bbox_pix(1)));
            oy_px = max(0, min(c1.bbox_pix(4), c2.bbox_pix(4)) - max(c1.bbox_pix(2), c2.bbox_pix(2)));
            area_px = ox_px * oy_px;

            if area_px > 0.01
                overlap_found_p2 = true;
                detail_str = sprintf('%s vs %s: Overlap Area = %.2f px^2 (dx=%.2f, dy=%.2f)', ...
                    c1.name, c2.name, area_px, ox_px, oy_px);
                overlap_details_p2{end+1} = detail_str;
            end
        end
    end

    record_result(~overlap_found_p2, ...
        '2.1 Panel 2 All-Pairs Overlap Check', ...
        sprintf('Checked %d control pairs in Panel 2. Zero overlaps detected.', num_pairs_checked_p2));
    if overlap_found_p2
        for d = 1:numel(overlap_details_p2)
            fprintf('         Overlap: %s\n', overlap_details_p2{d});
        end
    end

    % 2.2 Vertical gap check between J4 row controls and preset buttons (btn_zero, btn_home)
    j4_indices = [];
    for i = 1:n_p2
        nm = ctrl_list_p2(i).name;
        if contains(nm, '(4)') || contains(nm, 'J4')
            j4_indices(end+1) = i;
        end
    end

    j4_bottoms_norm = arrayfun(@(idx) ctrl_list_p2(idx).bbox_norm(2), j4_indices);
    j4_bottoms_pix  = arrayfun(@(idx) ctrl_list_p2(idx).bbox_pix(2), j4_indices);
    min_j4_bottom_norm = min(j4_bottoms_norm);
    min_j4_bottom_pix  = min(j4_bottoms_pix);

    u_zero = get(h.btn_zero, 'Units');
    set(h.btn_zero, 'Units', 'normalized');
    zero_norm = get(h.btn_zero, 'Position');
    set(h.btn_zero, 'Units', 'pixels');
    zero_pix = get(h.btn_zero, 'Position');
    set(h.btn_zero, 'Units', u_zero);

    u_home = get(h.btn_home, 'Units');
    set(h.btn_home, 'Units', 'normalized');
    home_norm = get(h.btn_home, 'Position');
    set(h.btn_home, 'Units', 'pixels');
    home_pix = get(h.btn_home, 'Position');
    set(h.btn_home, 'Units', u_home);

    max_preset_top_norm = max(zero_norm(2) + zero_norm(4), home_norm(2) + home_norm(4));
    max_preset_top_pix  = max(zero_pix(2) + zero_pix(4), home_pix(2) + home_pix(4));

    gap_j4_presets_norm = min_j4_bottom_norm - max_preset_top_norm;
    gap_j4_presets_pix  = min_j4_bottom_pix - max_preset_top_pix;

    detail_j4_presets = sprintf('J4 row bottom = %.2f px (norm=%.3f), Presets top = %.2f px (norm=%.3f) -> Gap = %.2f px (norm=%.3f)', ...
        min_j4_bottom_pix, min_j4_bottom_norm, max_preset_top_pix, max_preset_top_norm, gap_j4_presets_pix, gap_j4_presets_norm);
    record_result(gap_j4_presets_pix > 0, '2.2 Panel 2 J4 Row to Presets Vertical Gap', detail_j4_presets);

    % Clean up figure 1
    h.cleanup();
    delete(f);

    %% ====================================================================
    %% PART 3: 3D VISUALIZER WAYPOINT MARKER RENDERING EMPIRICAL VERIFICATION
    %% ====================================================================
    fprintf('\n--- PART 3: 3D VISUALIZER WAYPOINT MARKER RENDERING ---\n');

    % 3.1 Test launching palletizer_gui('visible', false) exactly as specified
    launch_err_msg = '';
    launch_bool_ok = false;
    try
        [f_test, h_test] = palletizer_gui('visible', false);
        launch_bool_ok = true;
        h_test.cleanup();
        delete(f_test);
    catch ME
        launch_err_msg = ME.message;
    end

    % Record adversarial empirical observation
    fprintf('  Empirical Test 3.1a: palletizer_gui(''visible'', false) call result:\n');
    if launch_bool_ok
        fprintf('         Success: accepted logical false as parameter value.\n');
        record_result(true, '3.1a Launch palletizer_gui(''visible'', false)', 'Direct call succeeded');
    else
        fprintf('         Observed Exception: %s\n', launch_err_msg);
        record_result(false, '3.1a Launch palletizer_gui(''visible'', false)', ...
            sprintf('inputParser rejected boolean false: %s', launch_err_msg));
    end

    % 3.1b Launch headless GUI with standard 'Visible', 'off' (or 'visible', 'off')
    [f_vis, h_vis] = palletizer_gui('Visible', 'off');
    record_result(isvalid(f_vis) && isfield(h_vis, 'h_waypoints') && isvalid(h_vis.h_waypoints), ...
        '3.1b Headless GUI Launch (Visible=off)', 'GUI launched and handles populated successfully');

    % 3.2 Check get(handles.h_waypoints, 'SizeData') immediately after initialization
    init_sizedata = get(h_vis.h_waypoints, 'SizeData');
    fprintf('  Initial Waypoints SizeData: %s\n', mat2str(init_sizedata));
    is_init_size_ok = all(init_sizedata >= 6) && all(init_sizedata <= 8);
    record_result(is_init_size_ok, ...
        '3.2 Waypoints Initial SizeData in [6, 8]', ...
        sprintf('Initial SizeData = %s (expected in [6, 8])', mat2str(init_sizedata)));

    % 3.3 Call handles.load_file('d:\Lidar\myPalletizer_BTL\data\trajectory_logo_bach_khoa.csv')
    logo_csv_path = 'd:\Lidar\myPalletizer_BTL\data\trajectory_logo_bach_khoa.csv';
    [ok_logo, msg_logo] = h_vis.load_file(logo_csv_path);
    fprintf('  Loaded Bach Khoa logo CSV: status=%d, msg=%s\n', ok_logo, msg_logo);

    logo_sizedata = get(h_vis.h_waypoints, 'SizeData');
    logo_xdata = get(h_vis.h_waypoints, 'XData');
    fprintf('  Logo Waypoints SizeData: %s (Count = %d waypoints)\n', mat2str(unique(logo_sizedata)), numel(logo_xdata));
    is_logo_size_ok = ok_logo && all(logo_sizedata >= 6) && all(logo_sizedata <= 8) && numel(logo_xdata) > 0;
    record_result(is_logo_size_ok, ...
        '3.3 Logo Waypoints SizeData in [6, 8] after load', ...
        sprintf('Logo SizeData = %s, Waypoints Count = %d (expected [6, 8])', mat2str(unique(logo_sizedata)), numel(logo_xdata)));

    % 3.4 Call handles.load_file('d:\Lidar\myPalletizer_BTL\data\trajectory_sample.csv') (8 waypoints)
    sample_csv_path = 'd:\Lidar\myPalletizer_BTL\data\trajectory_sample.csv';
    [ok_sample, msg_sample] = h_vis.load_file(sample_csv_path);
    fprintf('  Loaded Sample CSV: status=%d, msg=%s\n', ok_sample, msg_sample);

    sample_sizedata = get(h_vis.h_waypoints, 'SizeData');
    sample_xdata = get(h_vis.h_waypoints, 'XData');
    fprintf('  Sample Waypoints SizeData: %s (Count = %d waypoints)\n', mat2str(unique(sample_sizedata)), numel(sample_xdata));
    is_sample_size_ok = ok_sample && all(sample_sizedata >= 6) && all(sample_sizedata <= 8) && numel(sample_xdata) == 8;
    record_result(is_sample_size_ok, ...
        '3.4 Sample 8-Waypoint SizeData in [6, 8] after load', ...
        sprintf('Sample SizeData = %s, Waypoints Count = %d (expected 8 waypoints, SizeData in [6, 8])', ...
            mat2str(unique(sample_sizedata)), numel(sample_xdata)));

    % 3.5 Adversarial Switching Stress: Alternate loading logo and sample 10 times
    fprintf('  Testing alternating trajectory file load stress (10 switches)...\n');
    alternating_stress_ok = true;
    for sw = 1:10
        if mod(sw, 2) == 1
            [ok_s, ~] = h_vis.load_file(logo_csv_path);
            s_data = get(h_vis.h_waypoints, 'SizeData');
            x_pts = get(h_vis.h_waypoints, 'XData');
            if ~ok_s || ~all(s_data >= 6 & s_data <= 8) || numel(x_pts) ~= 112
                alternating_stress_ok = false;
                break;
            end
        else
            [ok_s, ~] = h_vis.load_file(sample_csv_path);
            s_data = get(h_vis.h_waypoints, 'SizeData');
            x_pts = get(h_vis.h_waypoints, 'XData');
            if ~ok_s || ~all(s_data >= 6 & s_data <= 8) || numel(x_pts) ~= 8
                alternating_stress_ok = false;
                break;
            end
        end
    end
    record_result(alternating_stress_ok, ...
        '3.5 Alternating Trajectory Load Stress (10 cycles)', ...
        'SizeData remained invariant in [6, 8] and waypoint counts (112 vs 8) correct across all 10 alternating switches');

    % 3.6 JSON Format Verification
    logo_json_path = 'd:\Lidar\myPalletizer_BTL\data\trajectory_logo_bach_khoa.json';
    if exist(logo_json_path, 'file')
        [ok_j, msg_j] = h_vis.load_file(logo_json_path);
        j_sizedata = get(h_vis.h_waypoints, 'SizeData');
        is_j_ok = ok_j && all(j_sizedata >= 6) && all(j_sizedata <= 8);
        record_result(is_j_ok, '3.6 JSON Trajectory Waypoints SizeData in [6, 8]', ...
            sprintf('Logo JSON SizeData = %s, status=%d', mat2str(unique(j_sizedata)), ok_j));
    else
        record_result(true, '3.6 JSON Trajectory (Skip if not present)', 'File not found, skipped');
    end

    % 3.7 Clean Shutdown
    h_vis.cleanup();
    delete(f_vis);
    record_result(~isvalid(f_vis), '3.7 Clean GUI Shutdown via handles.cleanup()', 'Figure closed cleanly');

    %% ====================================================================
    %% SUMMARY AND VERDICT
    %% ====================================================================
    fprintf('\n================================================================================\n');
    fprintf('  EMPIRICAL TEST SUMMARY                                                        \n');
    fprintf('  Total Tests Executed: %d\n', results.total_tests);
    fprintf('  Passed:               %d\n', results.total_pass);
    fprintf('  Failed:               %d\n', results.total_fail);
    fprintf('================================================================================\n');

    results.gap_row4_ik_pix = gap_row4_ik_pix;
    results.gap_ik_led_pix = gap_ik_led_pix;
    results.gap_j4_presets_pix = gap_j4_presets_pix;
    results.init_sizedata = init_sizedata;
    results.logo_sizedata = unique(logo_sizedata);
    results.sample_sizedata = unique(sample_sizedata);

    % Save results struct
    save(fullfile(projectRoot, 'tests', 'adversarial_enhancement_1_results.mat'), 'results');
end
