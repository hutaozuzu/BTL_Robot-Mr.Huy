% TEST_CHALLENGER_M3_2 Empirical Adversarial Stress Test Suite
% Milestone 3: Trajectory Planning & Palletizing Motion Controller
% Focus: Joint Velocity Limit Enforcement, Dynamic Duration Stretching,
%        Hardware Boundary Compliance, and Edge Cases.
%
% Challenger: teamwork_challenger (Challenger M3-2)
% Date: 2026-11-07

fprintf('================================================================================\n');
fprintf('  ADVERSARIAL STRESS TEST SUITE: TRAJECTORY VELOCITY LIMITS & STRETCHING       \n');
fprintf('  Challenger: teamwork_challenger (Challenger M3-2)                             \n');
fprintf('  Robot: 4-DOF myPalletizer 260 Pi | Pure Native MATLAB                         \n');
fprintf('================================================================================\n\n');

projectRoot = 'd:\Lidar\myPalletizer_BTL';
addpath(fullfile(projectRoot, 'kinematics'));
addpath(fullfile(projectRoot, 'trajectory'));
addpath(fullfile(projectRoot, 'tests'));

MAX_VEL_RAD = deg2rad(120.0); % 2.0943951 rad/s
HW_LIMITS_DEG = [ ...
    -162.0,  162.0;   % Joint 1: Base
      -2.0,   90.0;   % Joint 2: Shoulder
     -92.0,   60.0;   % Joint 3: Elbow
    -180.0,  180.0    % Joint 4: Wrist
];
HW_LIMITS_RAD = deg2rad(HW_LIMITS_DEG);

tests_passed = 0;
tests_failed = 0;
total_tests = 10;
test_records = repmat(struct('ID', 0, 'Name', '', 'Pass', false, 'Details', ''), total_tests, 1);

%% -----------------------------------------------------------------------------
%% TEST 1: Large Joint 1 Displacement (~300 deg) with Aggressive T_req = 0.5 s
%% -----------------------------------------------------------------------------
test_id = 1;
test_name = 'Large Joint 1 (~300 deg) displacement with aggressive T_req = 0.5s';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % q1 moves from -150 deg to +150 deg (delta = 300 deg = 5.236 rad)
    q_start = deg2rad([-150.0, 0.0, 0.0, 0.0]);
    q_end   = deg2rad([ 150.0, 0.0, 0.0, 0.0]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    traj = multi_point_planner(wps, T_req, 0.01, 'joint');
    
    % Unstretched peak vel would be: 1.875 * 5.235988 / 0.5 = 19.635 rad/s (1125 deg/s)
    expected_unstretched_v = 1.875 * deg2rad(300.0) / T_req;
    stretched_T = traj.segment_durations(1);
    min_theoretical_T = (1.875 * deg2rad(300.0) / MAX_VEL_RAD) * 1.05;
    
    peak_vel = max(abs(traj.dtheta(:)));
    peak_vel_deg = rad2deg(peak_vel);
    
    % Check stretching occurred
    assert(stretched_T >= min_theoretical_T - 1e-4, ...
        sprintf('Duration not properly stretched: T=%.3fs < min_theoretical=%.3fs', stretched_T, min_theoretical_T));
    assert(stretched_T > T_req * 8, ...
        sprintf('Duration should be ~10x T_req, got T=%.3fs', stretched_T));
    
    % Check peak velocity strictly <= 120 deg/s
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Peak velocity exceeded: %.4f rad/s (%.2f deg/s) > %.4f rad/s', peak_vel, peak_vel_deg, MAX_VEL_RAD));
    
    % Check hardware limits across full trajectory
    assert(traj.is_valid, 'Trajectory is_valid returned false.');
    q1_min_deg = rad2deg(min(traj.theta(:, 1)));
    q1_max_deg = rad2deg(max(traj.theta(:, 1)));
    assert(q1_min_deg >= -162.0 - 1e-4 && q1_max_deg <= 162.0 + 1e-4, ...
        sprintf('Joint 1 out of bounds: [%.2f, %.2f] deg', q1_min_deg, q1_max_deg));
    
    details_str = sprintf('T_req=%.2fs -> stretched to T=%.3fs (min=%.3fs). Peak vel=%.2f deg/s (limit 120 deg/s). Q1 in [%.1f, %.1f] deg.', ...
        T_req, stretched_T, min_theoretical_T, peak_vel_deg, q1_min_deg, q1_max_deg);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 2: Boundary Extreme Displacement (320 deg) with Ultra-Aggressive T_req = 0.2 s
%% -----------------------------------------------------------------------------
test_id = 2;
test_name = 'Boundary Extreme (320 deg) displacement with ultra-aggressive T_req = 0.2s';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % q1 from -160 deg to +160 deg (delta = 320 deg, close to max 324 deg)
    q_start = deg2rad([-160.0, 10.0, -10.0, 0.0]);
    q_end   = deg2rad([ 160.0, 10.0, -10.0, 0.0]);
    wps = [q_start; q_end];
    T_req = 0.2; % 16x overspeed demand
    
    traj = multi_point_planner(wps, T_req, 0.01, 'joint');
    
    stretched_T = traj.segment_durations(1);
    min_theoretical_T = (1.875 * deg2rad(320.0) / MAX_VEL_RAD) * 1.05;
    peak_vel = max(abs(traj.dtheta(:)));
    peak_vel_deg = rad2deg(peak_vel);
    
    assert(stretched_T >= min_theoretical_T - 1e-4, ...
        sprintf('Duration stretching insufficient: T=%.3fs < min=%.3fs', stretched_T, min_theoretical_T));
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Peak velocity exceeded: %.4f rad/s (%.2f deg/s) > 120 deg/s', peak_vel, peak_vel_deg));
    assert(traj.is_valid, 'Trajectory is_valid returned false.');
    
    % Verify no overshoot outside [-162, 162]
    q1_min_deg = rad2deg(min(traj.theta(:, 1)));
    q1_max_deg = rad2deg(max(traj.theta(:, 1)));
    assert(q1_min_deg >= -162.0 - 1e-4 && q1_max_deg <= 162.0 + 1e-4, ...
        sprintf('Overshoot detected in Joint 1: [%.3f, %.3f] deg', q1_min_deg, q1_max_deg));
    
    details_str = sprintf('T_req=%.2fs -> stretched to T=%.3fs. Peak vel=%.2f deg/s. Bounds=[%.2f, %.2f] deg strictly inside [-162, 162].', ...
        T_req, stretched_T, peak_vel_deg, q1_min_deg, q1_max_deg);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 3: Multi-Joint Simultaneous Extreme Displacements (All 4 joints near max span)
%% -----------------------------------------------------------------------------
test_id = 3;
test_name = 'Multi-joint simultaneous extreme displacements with aggressive T_req = 0.5s';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % Joint 1: -150 -> +150 (300 deg span)
    % Joint 2:    0 ->  +85 ( 85 deg span, limits [-2, 90])
    % Joint 3:  -85 ->  +50 (135 deg span, limits [-92, 60])
    % Joint 4: -170 -> +170 (340 deg span, limits [-180, 180])
    q_start = deg2rad([-150.0,  0.0, -85.0, -170.0]);
    q_end   = deg2rad([ 150.0, 85.0,  50.0,  170.0]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    traj = multi_point_planner(wps, T_req, 0.01, 'joint');
    
    % Joint 4 has the largest displacement: 340 deg
    max_span_rad = deg2rad(340.0);
    min_theoretical_T = (1.875 * max_span_rad / MAX_VEL_RAD) * 1.05;
    stretched_T = traj.segment_durations(1);
    
    assert(stretched_T >= min_theoretical_T - 1e-4, ...
        sprintf('Duration stretching did not account for governing joint (J4): T=%.3fs < min=%.3fs', ...
        stretched_T, min_theoretical_T));
    
    % Verify peak velocity on every single joint
    peak_vels = max(abs(traj.dtheta), [], 1);
    for j = 1:4
        assert(peak_vels(j) <= MAX_VEL_RAD + 1e-4, ...
            sprintf('Joint %d velocity exceeded: %.4f rad/s (%.2f deg/s) > 120 deg/s', ...
            j, peak_vels(j), rad2deg(peak_vels(j))));
    end
    
    % Verify hardware bounds on all 4 joints
    for j = 1:4
        q_min_j = min(traj.theta(:, j));
        q_max_j = max(traj.theta(:, j));
        assert(q_min_j >= HW_LIMITS_RAD(j, 1) - 1e-4 && q_max_j <= HW_LIMITS_RAD(j, 2) + 1e-4, ...
            sprintf('Joint %d hardware limit violated: [%.2f, %.2f] deg vs limits [%.2f, %.2f] deg', ...
            j, rad2deg(q_min_j), rad2deg(q_max_j), HW_LIMITS_DEG(j, 1), HW_LIMITS_DEG(j, 2)));
    end
    assert(traj.is_valid, 'Trajectory is_valid returned false.');
    
    details_str = sprintf('Governing J4 (340 deg) stretched T to %.3fs. Peak vels=[%.1f, %.1f, %.1f, %.1f] deg/s (all <= 120 deg/s). All 4 joints inside hardware bounds.', ...
        stretched_T, rad2deg(peak_vels(1)), rad2deg(peak_vels(2)), rad2deg(peak_vels(3)), rad2deg(peak_vels(4)));
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 4: Multi-Segment Heterogeneous Mix (Short segments vs Stretched segments)
%% -----------------------------------------------------------------------------
test_id = 4;
test_name = 'Multi-segment heterogeneous mix (selective duration stretching per segment)';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % 5 waypoints = 4 segments:
    % Seg 1: Small displacement (10 deg), T_req = 1.0 s -> No stretch needed
    % Seg 2: Huge J1 displacement (280 deg), T_req = 0.5 s -> Stretch needed
    % Seg 3: Huge J4 displacement (300 deg), T_req = 0.5 s -> Stretch needed
    % Seg 4: Small displacement (5 deg), T_req = 1.0 s -> No stretch needed
    wps = [ ...
        deg2rad([   0,  10, -20,   0]); ... % WP 1
        deg2rad([  10,  15, -25,   5]); ... % WP 2 (Seg 1: tiny)
        deg2rad([ 290 - 360, 20, -30, 10]); ... % Wait: 290 deg is -70 deg, span from 10 to -70 is 80 deg.
        % Let's make span 280 deg: from 140 deg to -140 deg!
    ];
    wps = [ ...
        deg2rad([   0,  20, -20,   0]); ... % WP 1
        deg2rad([  10,  20, -20,   0]); ... % WP 2 (Seg 1: delta J1 = 10 deg)
        deg2rad([ 150,  20, -20,   0]); ... % WP 3 (Seg 2: delta J1 = 140 deg)
        deg2rad([-140,  20, -20,   0]); ... % WP 4 (Seg 3: delta J1 = 290 deg from 150 to -140)
        deg2rad([-145,  20, -20,   5])  ... % WP 5 (Seg 4: delta J1 = 5 deg, delta J4 = 5 deg)
    ];
    T_req = [1.0; 0.5; 0.5; 1.0];
    
    traj = multi_point_planner(wps, T_req, 0.01, 'joint');
    durs = traj.segment_durations;
    
    % Seg 1: 10 deg in 1.0s -> max vel = 1.875 * 10 / 1.0 = 18.75 deg/s <= 120 -> no stretch
    assert(abs(durs(1) - 1.0) < 1e-6, sprintf('Seg 1 stretched unnecessarily: %.3fs', durs(1)));
    
    % Seg 2: 140 deg in 0.5s -> needs stretch to ~ 1.875*140/120 * 1.05 = 2.296s
    assert(durs(2) > 2.0, sprintf('Seg 2 not stretched enough: %.3fs', durs(2)));
    
    % Seg 3: 290 deg in 0.5s -> needs stretch to ~ 1.875*290/120 * 1.05 = 4.758s
    assert(durs(3) > 4.5, sprintf('Seg 3 not stretched enough: %.3fs', durs(3)));
    
    % Seg 4: 5 deg in 1.0s -> no stretch
    assert(abs(durs(4) - 1.0) < 1e-6, sprintf('Seg 4 stretched unnecessarily: %.3fs', durs(4)));
    
    % Check peak velocity across the whole trajectory
    peak_vel = max(abs(traj.dtheta(:)));
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Peak velocity across heterogeneous trajectory exceeded: %.4f rad/s > 120 deg/s', peak_vel));
    
    % Check continuity across all 4 segment boundaries
    seg_ends = cumsum(durs);
    for k = 1:(numel(seg_ends) - 1)
        tb = seg_ends(k);
        idx_b = find(traj.t <= tb, 1, 'last');
        idx_a = find(traj.t >= tb, 1, 'first');
        v_jump = max(abs(traj.dtheta(idx_a, :) - traj.dtheta(idx_b, :)));
        a_jump = max(abs(traj.ddtheta(idx_a, :) - traj.ddtheta(idx_b, :)));
        assert(v_jump < 1e-3, sprintf('Boundary %d velocity jump = %.2e >= 1e-3', k, v_jump));
        assert(a_jump < 1e-3, sprintf('Boundary %d acceleration jump = %.2e >= 1e-3', k, a_jump));
    end
    
    details_str = sprintf('Durations: [%.2f, %.2f, %.2f, %.2f]s (selective stretching verified). Max boundary vel jump < 1e-3, peak vel=%.2f deg/s.', ...
        durs(1), durs(2), durs(3), durs(4), rad2deg(peak_vel));
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 5: Aggressive Cartesian 8-Step Pick-and-Place Cycle (All T_req = 0.5 s)
%% -----------------------------------------------------------------------------
test_id = 5;
test_name = 'Aggressive Cartesian 8-step pick-and-place cycle (All T_req = 0.5s)';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0; % P1: Home
        220.0, -100.0, 120.0, 0.0; % P2: Pick Hover
        220.0, -100.0,  40.0, 0.0; % P3: Pick Point
        220.0, -100.0, 120.0, 0.0; % P4: Pick Retract
        180.0,  160.0, 140.0, 0.0; % P5: Transit
        180.0,  160.0,  70.0, 0.0; % P6: Place Point
        180.0,  160.0, 140.0, 0.0; % P7: Place Retract
        240.0,    0.0, 180.0, 0.0  % P8: Return Home
    ];
    T_req_all = 0.5 * ones(7, 1); % User requests aggressive 0.5s for every step (total nominal 3.5s)
    
    traj = multi_point_planner(wp_8step, T_req_all, 0.01, 'cartesian');
    
    % Transit from pick retract (220, -100, 120) to transit (180, 160, 140) spans ~70 deg in base yaw
    % Must stretch transit segment
    peak_vel = max(abs(traj.dtheta(:)));
    peak_vel_deg = rad2deg(peak_vel);
    
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Peak velocity exceeded in aggressive Cartesian cycle: %.4f rad/s (%.2f deg/s) > 120 deg/s', ...
        peak_vel, peak_vel_deg));
    assert(traj.total_duration > 3.5, ...
        sprintf('Total duration was not stretched: %.2fs <= 3.5s', traj.total_duration));
    assert(traj.is_valid, 'Trajectory is_valid returned false for reachable waypoints.');
    
    % Verify joint hardware limits along entire path
    for j = 1:4
        q_min = min(traj.theta(:, j));
        q_max = max(traj.theta(:, j));
        assert(q_min >= HW_LIMITS_RAD(j, 1) - 1e-4 && q_max <= HW_LIMITS_RAD(j, 2) + 1e-4, ...
            sprintf('Joint %d limit violated in Cartesian motion: [%.2f, %.2f] deg', ...
            j, rad2deg(q_min), rad2deg(q_max)));
    end
    
    details_str = sprintf('Nominal T=3.5s dynamically stretched to %.2fs. Peak vel=%.2f deg/s (<= 120 deg/s). All joints legal.', ...
        traj.total_duration, peak_vel_deg);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 6: Auto-Adjust Duration Disabled (auto_adjust_duration = false)
%% -----------------------------------------------------------------------------
test_id = 6;
test_name = 'auto_adjust_duration = false flags trajectory as invalid when limits violated';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % Large displacement 300 deg in 0.5s with auto_adjust_duration = false
    q_start = deg2rad([-150.0, 0.0, 0.0, 0.0]);
    q_end   = deg2rad([ 150.0, 0.0, 0.0, 0.0]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    opts = struct('auto_adjust_duration', false);
    traj = multi_point_planner(wps, T_req, 0.01, 'joint', opts);
    
    % Must keep original duration T_req = 0.5s, but flag is_valid = false
    assert(abs(traj.total_duration - T_req) < 1e-6, ...
        sprintf('Duration was modified despite auto_adjust_duration=false: %.3fs', traj.total_duration));
    assert(traj.is_valid == false, ...
        'Expected is_valid=false when auto_adjust_duration=false and velocity exceeds limit.');
    
    details_str = sprintf('auto_adjust_duration=false correctly preserved T=%.2fs and flagged is_valid=false.', traj.total_duration);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 7: Zero Displacement Edge Case with Aggressive Duration (T_req = 0.05 s)
%% -----------------------------------------------------------------------------
test_id = 7;
test_name = 'Zero displacement waypoint stationary hold with aggressive T_req = 0.05s';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    q_hold = deg2rad([45.0, 30.0, -45.0, 15.0]);
    wps = [q_hold; q_hold];
    T_req = 0.05;
    
    traj = multi_point_planner(wps, T_req, 0.01, 'joint');
    
    assert(traj.is_valid, 'Stationary hold flagged as invalid.');
    assert(~any(isnan(traj.theta(:))), 'NaNs detected in position.');
    assert(~any(isnan(traj.dtheta(:))), 'NaNs detected in velocity.');
    assert(~any(isnan(traj.ddtheta(:))), 'NaNs detected in acceleration.');
    
    peak_vel = max(abs(traj.dtheta(:)));
    assert(peak_vel < 1e-12, sprintf('Velocity is non-zero during stationary hold: %.2e', peak_vel));
    
    details_str = sprintf('Stationary hold across T=%.3fs completed with exact zero velocity (peak=%.2e rad/s). No NaNs.', ...
        traj.total_duration, peak_vel);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 8: Extreme Boundary Proximity Displacements (Touching hardware limits)
%% -----------------------------------------------------------------------------
test_id = 8;
test_name = 'Extreme boundary proximity displacements (Touching hardware limits)';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    % Waypoints placed at exactly 0.1 deg inside hardware limits:
    % J1: -161.9 -> +161.9 (delta = 323.8 deg)
    % J2:   -1.9 ->  +89.9 (delta =  91.8 deg)
    % J3:  -91.9 ->  +59.9 (delta = 151.8 deg)
    % J4: -179.9 -> +179.9 (delta = 359.8 deg)
    q_start = deg2rad([-161.9,  -1.9, -91.9, -179.9]);
    q_end   = deg2rad([ 161.9,  89.9,  59.9,  179.9]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    traj = multi_point_planner(wps, T_req, 0.005, 'joint'); % High resolution dt = 5ms
    
    assert(traj.is_valid, 'Near-limit trajectory flagged as invalid.');
    peak_vel = max(abs(traj.dtheta(:)));
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Peak velocity exceeded: %.4f rad/s > 120 deg/s', peak_vel));
    
    % Test every point for strict hardware limit containment (no overshoot!)
    for m = 1:numel(traj.t)
        for j = 1:4
            val_deg = rad2deg(traj.theta(m, j));
            assert(val_deg >= HW_LIMITS_DEG(j, 1) - 1e-4 && val_deg <= HW_LIMITS_DEG(j, 2) + 1e-4, ...
                sprintf('Sample %d (t=%.3fs): Joint %d overshot hardware limit! val=%.4f deg vs [%.1f, %.1f]', ...
                m, traj.t(m), j, val_deg, HW_LIMITS_DEG(j, 1), HW_LIMITS_DEG(j, 2)));
        end
    end
    
    stretched_T = traj.segment_durations(1);
    details_str = sprintf('Near-limit (J1:323.8 deg, J4:359.8 deg) stretched to %.3fs. Peak vel=%.2f deg/s. 0 overshoot across all %d samples.', ...
        stretched_T, rad2deg(peak_vel), numel(traj.t));
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 9: Interface Contract Wrapper plan_multi_point_trajectory Compliance
%% -----------------------------------------------------------------------------
test_id = 9;
test_name = 'Interface Contract plan_multi_point_trajectory inherits stretching and velocity limits';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    q_start = deg2rad([-150.0, 0.0, 0.0, 0.0]);
    q_end   = deg2rad([ 150.0, 0.0, 0.0, 0.0]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    % Call via interface contract wrapper from PROJECT.md
    traj = plan_multi_point_trajectory(wps, T_req, 0.02);
    
    assert(isfield(traj, 't') && isfield(traj, 'theta') && isfield(traj, 'dtheta') && ...
           isfield(traj, 'ddtheta') && isfield(traj, 'xyz_actual') && isfield(traj, 'is_valid'), ...
           'Interface wrapper returned struct missing required PROJECT.md fields.');
    
    peak_vel = max(abs(traj.dtheta(:)));
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
        sprintf('Wrapper failed velocity limit: %.4f rad/s > 120 deg/s', peak_vel));
    assert(traj.total_duration > T_req * 8, ...
        sprintf('Wrapper failed to stretch duration: T=%.3fs', traj.total_duration));
    assert(traj.is_valid, 'Wrapper returned is_valid=false.');
    
    details_str = sprintf('Interface wrapper conforms to PROJECT.md. Output fields valid, stretched T=%.3fs, peak vel=%.2f deg/s.', ...
        traj.total_duration, rad2deg(peak_vel));
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% TEST 10: Stretched Trajectory Simulation Streaming & Tracking Error
%% -----------------------------------------------------------------------------
test_id = 10;
test_name = 'Stretched high-speed trajectory streaming to simulate_trajectory.m';
fprintf('[TEST %02d] %s ...\n', test_id, test_name);
try
    q_start = deg2rad([-150.0, 10.0, -20.0, 30.0]);
    q_end   = deg2rad([ 150.0, 60.0,  20.0, -30.0]);
    wps = [q_start; q_end];
    T_req = 0.5;
    
    traj = multi_point_planner(wps, T_req, 0.02, 'joint');
    
    % Stream through simulate_trajectory
    [sim_res, stream_data] = simulate_trajectory(traj);
    
    assert(isfield(stream_data, 'time') && isfield(stream_data, 'signals'), ...
        'stream_data missing required Simulink From-Workspace fields.');
    assert(all(diff(stream_data.time) > 0), 'Streamer time vector is not strictly monotonic.');
    assert(sim_res.is_tracking_compliant, ...
        sprintf('Simulation tracking error non-compliant: max error = %.4f mm >= 1.0 mm', sim_res.max_cartesian_error));
    assert(sim_res.max_cartesian_error < 1.0, ...
        sprintf('Cartesian tracking error too high: %.4f mm', sim_res.max_cartesian_error));
    
    details_str = sprintf('Streamed %d points over %.3fs. Max tracking error=%.4f mm (< 1.0 mm), mean error=%.4f mm.', ...
        numel(stream_data.time), traj.total_duration, sim_res.max_cartesian_error, sim_res.mean_cartesian_error);
    fprintf('         -> PASS: %s\n', details_str);
    tests_passed = tests_passed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', true, 'Details', details_str);
catch ME
    fprintf('         -> FAIL: %s\n', ME.message);
    tests_failed = tests_failed + 1;
    test_records(test_id) = struct('ID', test_id, 'Name', test_name, 'Pass', false, 'Details', ME.message);
end

%% -----------------------------------------------------------------------------
%% SUMMARY & REPORT PERSISTENCE
%% -----------------------------------------------------------------------------
fprintf('\n================================================================================\n');
fprintf('  ADVERSARIAL STRESS TEST SUMMARY: %d / %d PASSED (%.1f%%)\n', ...
    tests_passed, total_tests, (tests_passed/total_tests)*100);
if tests_failed == 0
    fprintf('  VERDICT: >>> EMPIRICAL STRESS TESTS FULLY PASSED (APPROVE) <<<\n');
else
    fprintf('  VERDICT: >>> %d TEST(S) FAILED (REQUEST_CHANGES) <<<\n', tests_failed);
end
fprintf('================================================================================\n\n');

results_mat_path = fullfile(projectRoot, 'tests', 'adversarial_m3_2_results.mat');
save(results_mat_path, 'test_records', 'tests_passed', 'tests_failed', 'total_tests');
fprintf('Saved empirical results to: %s\n', results_mat_path);

if tests_failed > 0
    error('test_challenger_m3_2:TestsFailed', '%d of %d adversarial tests failed.', tests_failed, total_tests);
end
