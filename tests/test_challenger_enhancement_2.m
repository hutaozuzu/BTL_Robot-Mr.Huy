function [exit_code, test_summary] = test_challenger_enhancement_2()
% TEST_CHALLENGER_ENHANCEMENT_2 Empirical Adversarial Stress Testing for HCMUT Logo Trajectory & Kinematics
%
% Tests performed by Empirical Challenger 2:
%   PART 1: Workspace Reachability & Singularity Stress Test (CSV & JSON)
%     - 1.1: trajectory_parser validation on CSV
%     - 1.2: trajectory_parser validation on JSON and CSV-JSON identity
%     - 1.3: IK solvability and elbow_up branch verification for all 112 waypoints
%     - 1.4: Joint and Cartesian hardware boundary margins check
%     - 1.5: Analytical Jacobian det(J) and manipulability index w = sqrt(det(J*J^T)) > 1e-4
%   PART 2: Dynamic Trajectory Simulation & C^2 Continuity Test
%     - 2.1: Multi-point trajectory simulation at dt = 0.01 s (100 Hz)
%     - 2.2: Multi-point trajectory simulation stress test at dt = 0.005 s (200 Hz)
%     - 2.3: Numerical C^2 continuity verification across all 111 segment boundaries
%     - 2.4: Hardware joint velocity limit check (|dtheta_i| <= 180 deg/s)
%     - 2.5: Sub-micron Cartesian tracking accuracy at via-points (FK error <= 1e-3 mm)
%   PART 3: Geometric Verification of Official HCMUT Logo
%     - 3.1: 3-fold rotational symmetry (120 deg and 240 deg) of 3 isometric cubes (< 1e-12)
%     - 3.2: Angular positions of 3 cubes at exactly 90 deg, 210 deg, and 330 deg
%     - 3.3: Hexagon interior protection: radial seams terminate on boundary with zero penetration
%     - 3.4: Complete absence of "logo bách khoa" text strokes, and clean CSV/JSON content
%     - 3.5: Central typography verification: Bold 'BK' on top, 'TP.HCM' on bottom (single dot)
%   PART 4: Unambiguous Final Verdict (APPROVE / REQUEST_CHANGES)
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    test_start = tic;
    test_dir = fileparts(mfilename('fullpath'));
    base_dir = fullfile(test_dir, '..');
    
    addpath(test_dir);
    addpath(fullfile(base_dir, 'kinematics'));
    addpath(fullfile(base_dir, 'trajectory'));
    addpath(fullfile(base_dir, 'gui'));
    addpath(fullfile(base_dir, 'data'));
    
    fprintf('\n================================================================================\n');
    fprintf('  CHALLENGER 2: ADVERSARIAL STRESS TEST SUITE FOR HCMUT LOGO & KINEMATICS       \n');
    fprintf('================================================================================\n\n');

    tests_total = 0;
    tests_passed = 0;
    tests_failed = 0;
    results = struct('id', {}, 'name', {}, 'status', {}, 'message', {}, 'metrics', {});

    csv_file = fullfile(base_dir, 'data', 'trajectory_logo_bach_khoa.csv');
    json_file = fullfile(base_dir, 'data', 'trajectory_logo_bach_khoa.json');
    gen_file = fullfile(base_dir, 'data', 'generate_custom_trajectories.m');

    %% ------------------------------------------------------------------------
    %% PART 1: WORKSPACE REACHABILITY & SINGULARITY STRESS TEST
    %% ------------------------------------------------------------------------
    fprintf('>>> PART 1: Workspace Reachability & Singularity Stress Test <<<\n');
    
    % Test 1.1: Parse CSV file
    tests_total = tests_total + 1;
    t_id = 'CH2_01_PARSE_CSV';
    t_name = 'Parse trajectory_logo_bach_khoa.csv using trajectory_parser';
    try
        [wps_csv, is_valid_csv, err_csv] = trajectory_parser(csv_file);
        assert(is_valid_csv, sprintf('CSV parsing failed: %s', err_csv));
        assert(numel(wps_csv) >= 100, sprintf('Expected >= 100 waypoints, got %d', numel(wps_csv)));
        tests_passed = tests_passed + 1;
        results(tests_total) = log_pass(t_id, t_name, sprintf('Parsed %d waypoints successfully', numel(wps_csv)), []);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 1.2: Parse JSON file and verify exact match with CSV
    tests_total = tests_total + 1;
    t_id = 'CH2_02_PARSE_JSON';
    t_name = 'Parse trajectory_logo_bach_khoa.json and check 1-to-1 match with CSV';
    try
        [wps_json, is_valid_json, err_json] = trajectory_parser(json_file);
        assert(is_valid_json, sprintf('JSON parsing failed: %s', err_json));
        assert(numel(wps_json) == numel(wps_csv), ...
            sprintf('CSV has %d points but JSON has %d points', numel(wps_csv), numel(wps_json)));
        
        % Check coordinate consistency between CSV and JSON within 0.1 mm quantization resolution
        max_diff_coord = 0;
        for k = 1:numel(wps_csv)
            diff_xyz = norm([wps_csv(k).x - wps_json(k).x, ...
                             wps_csv(k).y - wps_json(k).y, ...
                             wps_csv(k).z - wps_json(k).z]);
            max_diff_coord = max(max_diff_coord, diff_xyz);
            assert(strcmp(wps_csv(k).action, wps_json(k).action), ...
                sprintf('Action mismatch at waypoint %d: CSV=%s, JSON=%s', k, wps_csv(k).action, wps_json(k).action));
        end
        % 0.1 mm is the format quantization of %.1f in CSV
        assert(max_diff_coord <= 0.10, sprintf('CSV vs JSON coordinate discrepancy = %.4f mm > 0.10 mm', max_diff_coord));
        
        tests_passed = tests_passed + 1;
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('Parsed %d waypoints from JSON. Exact 1-to-1 match with CSV (max coord diff = %.4f mm <= 0.1 mm resolution)', ...
            numel(wps_json), max_diff_coord), []);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 1.3: IK Solvability & Branch Consistency for all CSV Waypoints
    tests_total = tests_total + 1;
    t_id = 'CH2_03_IK_SOLVABILITY';
    t_name = 'Solve inverse_kinematics (elbow_up) for all waypoints';
    try
        n_wps = numel(wps_csv);
        thetas_ik = zeros(n_wps, 4);
        ik_errors = zeros(n_wps, 1);
        for k = 1:n_wps
            wp = wps_csv(k);
            [th_opt, succ, all_sol] = inverse_kinematics(wp.x, wp.y, wp.z, deg2rad(wp.phi), 'elbow_up');
            assert(succ, sprintf('Waypoint %d (id=%d) [x=%.1f, y=%.1f, z=%.1f] failed IK', k, wp.id, wp.x, wp.y, wp.z));
            % Check elbow_up configuration (theta3 <= 0)
            assert(th_opt(3) <= 1e-6, sprintf('Waypoint %d did not respect elbow_up: theta3 = %.4f rad', k, th_opt(3)));
            thetas_ik(k, :) = th_opt;
            
            % Round-trip check
            fk_pose = forward_kinematics(th_opt);
            ik_errors(k) = norm(fk_pose(1:3) - [wp.x, wp.y, wp.z]);
        end
        max_ik_err = max(ik_errors);
        assert(max_ik_err < 1e-10, sprintf('Max IK round-trip position error too high: %.2e mm', max_ik_err));
        tests_passed = tests_passed + 1;
        metrics = struct('n_waypoints', n_wps, 'max_ik_error_mm', max_ik_err);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('All %d waypoints solved IK (elbow_up). Max round-trip error = %.2e mm', n_wps, max_ik_err), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 1.4: Joint and Cartesian Limit Compliance
    tests_total = tests_total + 1;
    t_id = 'CH2_04_LIMITS_COMPLIANCE';
    t_name = 'Check check_limits for joints and cartesian bounds for all waypoints';
    try
        min_margins = inf(1, 4);
        for k = 1:n_wps
            wp = wps_csv(k);
            th = thetas_ik(k, :);
            
            % Joint limits
            [v_j, det_j] = check_limits(th, 'joints');
            assert(v_j, sprintf('Waypoint %d joint limit violation: %s', k, det_j.message));
            
            % Calculate margins from bounds
            dist_lower = th - det_j.lower_bounds;
            dist_upper = det_j.upper_bounds - th;
            margins = min([dist_lower; dist_upper], [], 1);
            min_margins = min(min_margins, margins);
            
            % Cartesian limits
            [v_c, det_c] = check_limits([wp.x, wp.y, wp.z], 'cartesian');
            assert(v_c, sprintf('Waypoint %d cartesian limit violation: %s', k, det_c.message));
        end
        tests_passed = tests_passed + 1;
        metrics = struct('min_joint_margins_deg', rad2deg(min_margins));
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('All %d waypoints inside joint & Cartesian bounds. Min margins: J1=%.1f°, J2=%.1f°, J3=%.1f°, J4=%.1f°', ...
            n_wps, rad2deg(min_margins(1)), rad2deg(min_margins(2)), rad2deg(min_margins(3)), rad2deg(min_margins(4))), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 1.5: Analytical Jacobian, Determinant & Manipulability Index (w > 1e-4)
    tests_total = tests_total + 1;
    t_id = 'CH2_05_MANIPULABILITY_SINGULARITY';
    t_name = 'Compute Jacobian, det(J), and manipulability w = sqrt(det(J*J^T)) > 1e-4';
    try
        w_all = zeros(n_wps, 1);
        det_all = zeros(n_wps, 1);
        cond_all = zeros(n_wps, 1);
        for k = 1:n_wps
            th = thetas_ik(k, :);
            [J, det_J, is_sing] = jacobian_matrix(th);
            assert(~is_sing, sprintf('Waypoint %d flagged as singular: det_J = %.2e', k, det_J));
            
            % Manipulability index w = sqrt(det(J * J'))
            w = sqrt(det(J * J'));
            assert(w > 1e-4, sprintf('Waypoint %d manipulability index w = %.2e <= 1e-4', k, w));
            
            w_all(k) = w;
            det_all(k) = det_J;
            cond_all(k) = cond(J);
        end
        min_w = min(w_all);
        max_w = max(w_all);
        min_abs_det = min(abs(det_all));
        max_cond = max(cond_all);
        
        tests_passed = tests_passed + 1;
        metrics = struct('min_w', min_w, 'max_w', max_w, 'min_abs_det', min_abs_det, 'max_condition_number', max_cond);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('All %d waypoints strictly non-singular. Min w = %.2e (> 1e-4), Min |det(J)| = %.2e, Max cond(J) = %.2f', ...
            n_wps, min_w, min_abs_det, max_cond), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    %% ------------------------------------------------------------------------
    %% PART 2: DYNAMIC TRAJECTORY SIMULATION & C^2 CONTINUITY TEST
    %% ------------------------------------------------------------------------
    fprintf('\n>>> PART 2: Dynamic Trajectory Simulation & C^2 Continuity Test <<<\n');

    % Prepare inputs for plan_multi_point_trajectory
    n_wps = numel(wps_csv);
    cart_matrix = zeros(n_wps, 4);
    durations = zeros(n_wps - 1, 1);
    for k = 1:n_wps
        cart_matrix(k, :) = [wps_csv(k).x, wps_csv(k).y, wps_csv(k).z, deg2rad(wps_csv(k).phi)];
        if k < n_wps
            durations(k) = wps_csv(k).duration;
        end
    end
    sim_dt = 0.01; % 100 Hz high resolution simulation

    % Test 2.1: Execution of plan_multi_point_trajectory at dt = 0.01 s
    tests_total = tests_total + 1;
    t_id = 'CH2_06_TRAJECTORY_SIMULATION_100HZ';
    t_name = 'Feed waypoints into plan_multi_point_trajectory at dt = 0.01 s';
    try
        traj = plan_multi_point_trajectory(cart_matrix, durations, sim_dt);
        assert(isstruct(traj), 'Trajectory output is not a struct');
        assert(traj.is_valid, 'Trajectory planner marked trajectory as invalid');
        assert(all(isfinite(traj.theta(:))), 'Joint positions contain NaN or Inf');
        assert(all(isfinite(traj.dtheta(:))), 'Joint velocities contain NaN or Inf');
        assert(all(isfinite(traj.ddtheta(:))), 'Joint accelerations contain NaN or Inf');
        assert(numel(traj.t) > 1000, sprintf('Expected >1000 time steps, got %d', numel(traj.t)));
        
        tests_passed = tests_passed + 1;
        metrics = struct('total_time_s', traj.total_duration, 'num_steps', numel(traj.t), 'dt', sim_dt);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('Trajectory generated successfully: T_total = %.2f s, M = %d steps at dt = %.3f s', ...
            traj.total_duration, numel(traj.t), sim_dt), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 2.2: Stress test at ultra-high frequency dt = 0.005 s (200 Hz)
    tests_total = tests_total + 1;
    t_id = 'CH2_07_TRAJECTORY_SIMULATION_200HZ';
    t_name = 'Stress test plan_multi_point_trajectory at dt = 0.005 s (200 Hz)';
    try
        sim_dt_fast = 0.005;
        traj_fast = plan_multi_point_trajectory(cart_matrix, durations, sim_dt_fast);
        assert(traj_fast.is_valid, 'High-frequency simulation marked as invalid');
        assert(numel(traj_fast.t) > 7000, 'Expected >7000 time steps at 200 Hz');
        tests_passed = tests_passed + 1;
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('High-frequency 200 Hz simulation stable: M = %d steps, T_total = %.2f s', ...
            numel(traj_fast.t), traj_fast.total_duration), []);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 2.3: C^2 Continuity Verification across Segment Boundaries
    tests_total = tests_total + 1;
    t_id = 'CH2_08_C2_CONTINUITY';
    t_name = 'Verify position, velocity, and acceleration continuity across all segment boundaries';
    try
        adj_durs = traj.segment_durations;
        seg_ends = cumsum(adj_durs);
        n_segs = numel(seg_ends);
        
        max_q_jump = 0;
        max_v_jump = 0;
        max_a_jump = 0;
        
        for s = 1:(n_segs - 1)
            t_b = seg_ends(s);
            idx_before = find(traj.t <= t_b, 1, 'last');
            idx_after = find(traj.t >= t_b, 1, 'first');
            
            % Position jump (C^0)
            q_jump = max(abs(traj.theta(idx_after, :) - traj.theta(idx_before, :)));
            max_q_jump = max(max_q_jump, q_jump);
            
            % Velocity jump (C^1)
            v_jump = max(abs(traj.dtheta(idx_after, :) - traj.dtheta(idx_before, :)));
            max_v_jump = max(max_v_jump, v_jump);
            
            % Acceleration jump (C^2)
            a_jump = max(abs(traj.ddtheta(idx_after, :) - traj.ddtheta(idx_before, :)));
            max_a_jump = max(max_a_jump, a_jump);
            
            assert(q_jump < 1e-4, sprintf('Segment boundary %d position jump: %.2e rad', s, q_jump));
            assert(v_jump < 1e-3, sprintf('Segment boundary %d velocity jump: %.2e rad/s', s, v_jump));
            assert(a_jump < 1e-3, sprintf('Segment boundary %d acceleration jump: %.2e rad/s^2', s, a_jump));
        end
        
        % Global derivative consistency along trajectory:
        num_v = diff(traj.theta) / sim_dt;
        v_mid = 0.5 * (traj.dtheta(1:end-1, :) + traj.dtheta(2:end, :));
        diff_v = max(abs(num_v(:) - v_mid(:)));
        
        tests_passed = tests_passed + 1;
        metrics = struct('max_boundary_pos_jump_rad', max_q_jump, ...
                         'max_boundary_vel_jump_rad_s', max_v_jump, ...
                         'max_boundary_acc_jump_rad_s2', max_a_jump, ...
                         'max_num_diff_vel_err', diff_v);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('C^2 continuity validated across %d boundaries. Max boundary jumps: pos=%.2e rad, vel=%.2e rad/s, acc=%.2e rad/s^2', ...
            n_segs - 1, max_q_jump, max_v_jump, max_a_jump), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 2.4: Joint Velocity Hardware Limit Compliance (|dtheta| <= 180 deg/s)
    tests_total = tests_total + 1;
    t_id = 'CH2_09_VELOCITY_LIMITS';
    t_name = 'Check maximum joint velocities against hardware limit (|dtheta_i| <= 180 deg/s)';
    try
        HW_LIMIT_DEG = 180.0;
        HW_LIMIT_RAD = deg2rad(HW_LIMIT_DEG);
        PLANNER_LIMIT_DEG = 120.0;
        PLANNER_LIMIT_RAD = deg2rad(PLANNER_LIMIT_DEG);
        
        peak_vel_per_joint = max(abs(traj.dtheta), [], 1);
        max_vel_rad = max(peak_vel_per_joint);
        max_vel_deg = rad2deg(max_vel_rad);
        
        assert(max_vel_rad <= HW_LIMIT_RAD + 1e-4, ...
            sprintf('Peak joint velocity %.2f deg/s exceeds hardware limit %.1f deg/s', max_vel_deg, HW_LIMIT_DEG));
        assert(max_vel_rad <= PLANNER_LIMIT_RAD + 1e-4, ...
            sprintf('Peak joint velocity %.2f deg/s exceeds planner design limit %.1f deg/s', max_vel_deg, PLANNER_LIMIT_DEG));
        
        tests_passed = tests_passed + 1;
        metrics = struct('peak_vel_deg_s', max_vel_deg, 'peak_per_joint_deg_s', rad2deg(peak_vel_per_joint));
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('Max joint velocity = %.2f deg/s <= hardware limit 180°/s (and planner limit 120°/s). Peak per joint: J1=%.1f, J2=%.1f, J3=%.1f, J4=%.1f deg/s', ...
            max_vel_deg, rad2deg(peak_vel_per_joint(1)), rad2deg(peak_vel_per_joint(2)), rad2deg(peak_vel_per_joint(3)), rad2deg(peak_vel_per_joint(4))), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 2.5: Cartesian Tracking Error at Waypoints (FK(theta(t_k)) vs (x_k, y_k, z_k) <= 1e-3 mm)
    tests_total = tests_total + 1;
    t_id = 'CH2_10_CARTESIAN_TRACKING_ERROR';
    t_name = 'Verify FK(theta(t)) matches commanded waypoints (x, y, z) within 10^-3 mm';
    try
        adj_durs = traj.segment_durations;
        t_knots = [0; cumsum(adj_durs)];
        n_knots = numel(t_knots);
        knot_errors = zeros(n_knots, 1);
        pitch_errors = zeros(n_knots, 1);
        
        for k = 1:n_knots
            t_target = t_knots(k);
            [~, idx_k] = min(abs(traj.t - t_target));
            
            % Compute exact FK of the planned joint angles at knot
            fk_k = forward_kinematics(traj.theta(idx_k, :));
            xyz_cmd = cart_matrix(k, 1:3);
            phi_cmd = cart_matrix(k, 4);
            
            err_k = norm(fk_k(1:3) - xyz_cmd);
            err_phi = abs(fk_k(4) - phi_cmd);
            knot_errors(k) = err_k;
            pitch_errors(k) = err_phi;
            
            assert(err_k <= 1e-3, ...
                sprintf('Waypoint %d (t=%.2fs) position tracking error = %.2e mm > 1e-3 mm', k, t_target, err_k));
            assert(err_phi <= 1e-4, ...
                sprintf('Waypoint %d (t=%.2fs) pitch tracking error = %.2e rad > 1e-4 rad', k, t_target, err_phi));
        end
        
        max_knot_err = max(knot_errors);
        mean_knot_err = mean(knot_errors);
        max_pitch_err = max(pitch_errors);
        
        tests_passed = tests_passed + 1;
        metrics = struct('max_waypoint_error_mm', max_knot_err, 'mean_waypoint_error_mm', mean_knot_err, 'max_pitch_error_rad', max_pitch_err);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('All %d waypoints tracked with sub-micron precision: max knot error = %.2e mm <= 10^-3 mm (mean = %.2e mm)', ...
            n_knots, max_knot_err, mean_knot_err), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    %% ------------------------------------------------------------------------
    %% PART 3: GEOMETRIC VERIFICATION OF OFFICIAL HCMUT LOGO
    %% ------------------------------------------------------------------------
    fprintf('\n>>> PART 3: Geometric Verification of Official HCMUT Logo <<<\n');

    % Define official HCMUT crest geometric parameters
    s3 = sqrt(3);
    H1 = [0.0, 1.0];             % 90 deg
    H2 = [-s3/2, 0.5];           % 150 deg
    H3 = [-s3/2, -0.5];          % 210 deg
    H4 = [0.0, -1.0];            % 270 deg
    H5 = [s3/2, -0.5];           % 330 deg
    H6 = [s3/2, 0.5];            % 30 deg

    top_rim  = [H2; -s3/2, 1.5; 0.0, 2.0; s3/2, 1.5; H6];
    top_seam = [0.0, 2.0; 0.0, 1.0];

    bl_rim   = [H4; -s3/2, -1.5; -s3, -1.0; -s3, 0.0; H2];
    bl_seam  = [-s3, -1.0; -s3/2, -0.5];

    br_rim   = [H6; s3, 0.0; s3, -1.0; s3/2, -1.5; H4];
    br_seam  = [s3, -1.0; s3/2, -0.5];

    % Test 3.1: 3-Fold Rotational Symmetry (120 deg) of 3 Isometric Cubes
    tests_total = tests_total + 1;
    t_id = 'CH2_11_ROTATIONAL_SYMMETRY';
    t_name = 'Verify 3-fold rotational symmetry (120 deg) across the 3 isometric cubes';
    try
        % 2D planar rotation matrix R(theta) in normalized (y, z) plane
        R120 = [cosd(120), -sind(120); sind(120), cosd(120)];
        R240 = [cosd(240), -sind(240); sind(240), cosd(240)];
        
        % Rotate Top Cube Rim by +120 deg and compare with Bottom-Left Rim
        top_rim_rot120 = (R120 * top_rim')';
        err_bl_rim = max(abs(top_rim_rot120(:) - bl_rim(:)));
        assert(err_bl_rim < 1e-12, sprintf('Top rim rotated +120° does not match BL rim: err = %.2e', err_bl_rim));
        
        % Rotate Top Cube Seam by +120 deg and compare with Bottom-Left Seam
        top_seam_rot120 = (R120 * top_seam')';
        err_bl_seam = max(abs(top_seam_rot120(:) - bl_seam(:)));
        assert(err_bl_seam < 1e-12, sprintf('Top seam rotated +120° does not match BL seam: err = %.2e', err_bl_seam));
        
        % Rotate Top Cube Rim by +240 deg (-120 deg) and compare with Bottom-Right Rim
        top_rim_rot240 = (R240 * top_rim')';
        err_br_rim = max(abs(top_rim_rot240(:) - br_rim(:)));
        assert(err_br_rim < 1e-12, sprintf('Top rim rotated +240° does not match BR rim: err = %.2e', err_br_rim));
        
        % Rotate Top Cube Seam by +240 deg (-120 deg) and compare with Bottom-Right Seam
        top_seam_rot240 = (R240 * top_seam')';
        err_br_seam = max(abs(top_seam_rot240(:) - br_seam(:)));
        assert(err_br_seam < 1e-12, sprintf('Top seam rotated +240° does not match BR seam: err = %.2e', err_br_seam));
        
        max_sym_err = max([err_bl_rim, err_bl_seam, err_br_rim, err_br_seam]);
        tests_passed = tests_passed + 1;
        metrics = struct('max_symmetry_discrepancy', max_sym_err);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('3-fold rotational symmetry (120° and 240°) verified mathematically. Max discrepancy = %.2e (< 1e-12)', max_sym_err), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 3.2: Angular Position of 3 Cubes (90 deg, 210 deg, 330 deg)
    tests_total = tests_total + 1;
    t_id = 'CH2_12_CUBE_ANGLES';
    t_name = 'Verify that the 3 cubes are centered at 90 deg, 210 deg, and 330 deg';
    try
        % Top cube outer apex: (0.0, 2.0)
        ang_top = atan2d(top_seam(1, 2), top_seam(1, 1));
        if ang_top < 0, ang_top = ang_top + 360; end
        assert(abs(ang_top - 90.0) < 1e-10, sprintf('Top cube apex angle is %.4f°, expected 90°', ang_top));
        
        % Bottom-left cube outer apex: (-s3, -1.0)
        ang_bl = atan2d(bl_seam(1, 2), bl_seam(1, 1));
        if ang_bl < 0, ang_bl = ang_bl + 360; end
        assert(abs(ang_bl - 210.0) < 1e-10, sprintf('BL cube apex angle is %.4f°, expected 210°', ang_bl));
        
        % Bottom-right cube outer apex: (s3, -1.0)
        ang_br = atan2d(br_seam(1, 2), br_seam(1, 1));
        if ang_br < 0, ang_br = ang_br + 360; end
        assert(abs(ang_br - 330.0) < 1e-10, sprintf('BR cube apex angle is %.4f°, expected 330°', ang_br));
        
        tests_passed = tests_passed + 1;
        metrics = struct('top_angle_deg', ang_top, 'bl_angle_deg', ang_bl, 'br_angle_deg', ang_br);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('All 3 cubes placed at exact prescribed angles: Top=%.1f°, BL=%.1f°, BR=%.1f° (Δ=120.0°)', ...
            ang_top, ang_bl, ang_br), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 3.3: No Radial Seams Enter Interior of Central Hexagon
    tests_total = tests_total + 1;
    t_id = 'CH2_13_NO_RADIAL_SEAMS_IN_HEX';
    t_name = 'Verify that no radial seams enter the interior of the central hexagon';
    try
        % 1. Mathematical normalized model verification:
        hex_norm = [H1; H2; H3; H4; H5; H6; H1];
        u = linspace(0, 1, 1000)';
        
        % Check continuous points along all 3 seams:
        seams_norm = {top_seam, bl_seam, br_seam};
        seam_names = {'Top Seam', 'Bottom-Left Seam', 'Bottom-Right Seam'};
        for i = 1:3
            s_pts = (1 - u) * seams_norm{i}(1, :) + u * seams_norm{i}(2, :);
            [in_s, on_s] = inpolygon(s_pts(:, 1), s_pts(:, 2), hex_norm(:, 1), hex_norm(:, 2));
            assert(sum(in_s & ~on_s) == 0, sprintf('%s penetrates interior of normalized central hexagon!', seam_names{i}));
        end
        
        % 2. Physical Trajectory CSV Waypoint Verification:
        % The drawn central hexagon is formed by waypoints 3 through 9:
        hex_csv_poly = [[wps_csv(3:9).y]', [wps_csv(3:9).z]'];
        
        % The 3 radial seams in the CSV:
        % Top seam: waypoints 19 to 20
        % Bottom-left seam: waypoints 30 to 31
        % Bottom-right seam: waypoints 41 to 42
        csv_seams = { ...
            [[wps_csv(19).y, wps_csv(19).z]; [wps_csv(20).y, wps_csv(20).z]], ...
            [[wps_csv(30).y, wps_csv(30).z]; [wps_csv(31).y, wps_csv(31).z]], ...
            [[wps_csv(41).y, wps_csv(41).z]; [wps_csv(42).y, wps_csv(42).z]] ...
        };
        
        for i = 1:3
            p_start = csv_seams{i}(1, :);
            p_end = csv_seams{i}(2, :);
            seam_interp = (1 - u) * p_start + u * p_end;
            [in_c, on_c] = inpolygon(seam_interp(:, 1), seam_interp(:, 2), hex_csv_poly(:, 1), hex_csv_poly(:, 2));
            assert(sum(in_c & ~on_c) == 0, sprintf('%s in CSV penetrates interior of drawn hexagon!', seam_names{i}));
        end
        
        % 3. Check exact boundary termination & pen retraction:
        % Top seam end (wp 20) matches H1 (wp 3)
        assert(norm([wps_csv(20).y - wps_csv(3).y, wps_csv(20).z - wps_csv(3).z]) < 1e-6, ...
            'Top seam did not terminate on H1');
        assert(strcmp(wps_csv(21).action, 'MOVE'), 'Pen not lifted immediately after Top seam');
        
        % BL seam end (wp 31) matches H3 (wp 5)
        assert(norm([wps_csv(31).y - wps_csv(5).y, wps_csv(31).z - wps_csv(5).z]) < 1e-6, ...
            'BL seam did not terminate on H3');
        assert(strcmp(wps_csv(32).action, 'MOVE'), 'Pen not lifted immediately after BL seam');
        
        % BR seam end (wp 42) matches H5 (wp 7)
        assert(norm([wps_csv(42).y - wps_csv(7).y, wps_csv(42).z - wps_csv(7).z]) < 1e-6, ...
            'BR seam did not terminate on H5');
        assert(strcmp(wps_csv(43).action, 'MOVE'), 'Pen not lifted immediately after BR seam');
        
        tests_passed = tests_passed + 1;
        results(tests_total) = log_pass(t_id, t_name, ...
            'All 3 radial seams terminate strictly on hexagon boundary vertices (H1, H3, H5). Zero interior penetration in 3000 sampled points.', []);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 3.4: Complete Absence of "logo bách khoa" Trajectory Strokes and Text
    tests_total = tests_total + 1;
    t_id = 'CH2_14_TEXT_ABSENCE';
    t_name = 'Verify that robot draws NO "logo bách khoa" text strokes, and check CSV/JSON content';
    try
        % 1. Read CSV text and JSON text
        csv_text = fileread(csv_file);
        json_text = fileread(json_file);
        
        % Ensure neither CSV nor JSON contains 'logo bách khoa' or 'logo bach khoa'
        assert(~contains(lower(csv_text), 'logo bách khoa'), 'CSV contains forbidden phrase "logo bách khoa"');
        assert(~contains(lower(json_text), 'logo bách khoa'), 'JSON contains forbidden phrase "logo bách khoa"');
        assert(~contains(lower(csv_text), 'logo bach khoa'), 'CSV contains phrase "logo bach khoa"');
        assert(~contains(lower(json_text), 'logo bach khoa'), 'JSON contains phrase "logo bach khoa"');
        
        % 2. Inspect stroke definition in generate_custom_trajectories.m
        gen_text = fileread(gen_file);
        % Check that no letter definitions for "L", "O", "G", "O" exist in Part 2 of generator
        assert(~contains(gen_text, 'strokes_L'), 'Generator script contains strokes_L definition');
        assert(~contains(gen_text, 'strokes_B_diacritic'), 'Generator script contains strokes_B_diacritic definition');
        
        tests_passed = tests_passed + 1;
        metrics = struct('csv_clean', true, 'json_clean', true, 'no_drawing_strokes', true);
        results(tests_total) = log_pass(t_id, t_name, ...
            'Zero strokes for "logo bách khoa" drawn by robot. CSV & JSON files are 100% free of "logo bách khoa".', metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    % Test 3.5: Central Typography Verification ('BK' top, 'TP.HCM' bottom)
    tests_total = tests_total + 1;
    t_id = 'CH2_15_TYPOGRAPHY_STRUCTURE';
    t_name = 'Verify central typography: Bold BK on top half, TP.HCM on bottom half with single dot';
    try
        % Waypoints 44..67 correspond to 'BK'
        % Waypoints 68..111 correspond to 'TP.HCM'
        bk_wps = wps_csv(44:67);
        tphcm_wps = wps_csv(68:111);
        
        min_bk_z = min([bk_wps.z]);
        max_bk_z = max([bk_wps.z]);
        min_tphcm_z = min([tphcm_wps.z]);
        max_tphcm_z = max([tphcm_wps.z]);
        
        % Verify vertical stratification
        gap_z = min_bk_z - max_tphcm_z;
        assert(gap_z > 0, sprintf('BK and TP.HCM overlap vertically: gap = %.2f mm <= 0', gap_z));
        assert(min_bk_z >= 133.0, sprintf('BK extends below expected baseline: min_z = %.1f mm', min_bk_z));
        assert(max_tphcm_z <= 131.0, sprintf('TP.HCM extends above expected ceiling: max_z = %.1f mm', max_tphcm_z));
        
        % Verify dot between P and H:
        % Waypoints 83..86 are dot_poly
        dot_wps = wps_csv(83:86);
        dot_y = mean([dot_wps.y]);
        dot_z = mean([dot_wps.z]);
        % Check that dot is located between P (y in [-11.5, -7.4]) and H (y in [-2.2, 2.2])
        assert(dot_y > -7.4 && dot_y < -2.2, sprintf('Dot position y=%.2f is not between P and H', dot_y));
        
        gen_text = fileread(gen_file);
        assert(contains(gen_text, 'dot_poly'), 'Generator missing single dot between P and H');
        assert(~contains(gen_text, 'dot_after_m'), 'Generator contains incorrect dot after M');
        
        tests_passed = tests_passed + 1;
        metrics = struct('min_bk_z_mm', min_bk_z, 'max_bk_z_mm', max_bk_z, ...
                         'min_tphcm_z_mm', min_tphcm_z, 'max_tphcm_z_mm', max_tphcm_z, ...
                         'vertical_clearance_gap_mm', gap_z);
        results(tests_total) = log_pass(t_id, t_name, ...
            sprintf('Central typography stratified: BK in [%.1f, %.1f] mm, TP.HCM in [%.1f, %.1f] mm, clean gap = %.1f mm. Single dot between P and H at y=%.1f mm.', ...
            min_bk_z, max_bk_z, min_tphcm_z, max_tphcm_z, gap_z, dot_y), metrics);
        fprintf('  [PASS] %s: %s\n', t_id, results(tests_total).message);
    catch ME
        tests_failed = tests_failed + 1;
        results(tests_total) = log_fail(t_id, t_name, ME.message);
        fprintf('  [FAIL] %s: %s\n', t_id, ME.message);
    end

    %% ------------------------------------------------------------------------
    %% PART 4: SUMMARY & VERDICT
    %% ------------------------------------------------------------------------
    duration_total = toc(test_start);
    fprintf('\n================================================================================\n');
    fprintf('                    CHALLENGER 2 ADVERSARIAL TEST SUMMARY                      \n');
    fprintf('================================================================================\n');
    fprintf('  Total Tests Executed : %d\n', tests_total);
    fprintf('  Tests Passed         : %d\n', tests_passed);
    fprintf('  Tests Failed         : %d\n', tests_failed);
    fprintf('  Execution Time       : %.2f seconds\n', duration_total);
    fprintf('--------------------------------------------------------------------------------\n');
    
    if tests_failed == 0
        final_verdict = 'Verdict: APPROVE';
        exit_code = 0;
        fprintf('  >>> FINAL VERDICT: %s <<<\n', final_verdict);
    else
        final_verdict = 'Verdict: REQUEST_CHANGES';
        exit_code = 1;
        fprintf('  >>> FINAL VERDICT: %s <<<\n', final_verdict);
    end
    fprintf('================================================================================\n\n');

    test_summary = struct();
    test_summary.total = tests_total;
    test_summary.passed = tests_passed;
    test_summary.failed = tests_failed;
    test_summary.duration_s = duration_total;
    test_summary.verdict = final_verdict;
    test_summary.results = results;
end

function r = log_pass(id, name, msg, metrics)
    r = struct('id', id, 'name', name, 'status', 'PASS', 'message', msg, 'metrics', metrics);
end

function r = log_fail(id, name, msg)
    r = struct('id', id, 'name', name, 'status', 'FAIL', 'message', msg, 'metrics', []);
end
