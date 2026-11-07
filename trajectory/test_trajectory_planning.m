% TEST_TRAJECTORY_PLANNING Automated unit test suite for Milestone 3.
%
% Verifies:
%   1. Quintic polynomial boundary conditions and C^2 continuity
%   2. Multi-point trajectory planner with C^2 continuity across segment boundaries (jump < 1e-3)
%   3. Joint velocity limits compliance (|dtheta| <= 120 deg/s = 2.0944 rad/s)
%   4. Pallet grid coordinate generation (2x2x3) and workspace compliance (R <= 260 mm)
%   5. Simulation trajectory streamer and tracking error (< 1.0 mm)
%   6. Zero Peter Corke Robotics Toolbox usage verification
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

clear; clc;
fprintf('================================================================================\n');
fprintf('  MILESTONE 3: TRAJECTORY PLANNING & MOTION CONTROLLER UNIT TEST SUITE\n');
fprintf('  4-DOF myPalletizer 260 Pi | Pure Native MATLAB | Zero Peter Corke Toolbox\n');
fprintf('================================================================================\n\n');

% Add kinematics and trajectory folders to MATLAB path
current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir), current_dir = pwd; end
kinematics_dir = fullfile(fileparts(current_dir), 'kinematics');
addpath(current_dir);
addpath(kinematics_dir);

tests_passed = 0;
tests_failed = 0;
total_tests = 15;

% Test 1: Quintic Polynomial Output Dimensions & Return Types
fprintf('[TEST 01] Quintic polynomial output dimensions and scalar/vector interfaces ... ');
try
    s0 = 0; sf = 100; t0 = 0; tf = 2.0; t = linspace(t0, tf, 50);
    [s, ds, dds] = quintic_trajectory(s0, sf, t0, tf, t);
    assert(numel(s) == 50 && numel(ds) == 50 && numel(dds) == 50, 'Scalar output dimension mismatch.');

    % Vector targets [1 x 4]
    q0 = [0.1, 0.2, -0.3, 0.4];
    qf = [0.5, -0.1, 0.3, -0.2];
    [s_vec, ds_vec, dds_vec] = quintic_trajectory(q0, qf, t0, tf, t);
    assert(isequal(size(s_vec), [50, 4]) && isequal(size(ds_vec), [50, 4]) && isequal(size(dds_vec), [50, 4]), ...
           'Vector output dimension mismatch.');

    fprintf('PASS\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 2: Quintic Polynomial Boundary Conditions
fprintf('[TEST 02] Quintic polynomial boundary conditions s(t0)=s0, s(tf)=sf, ds=0, dds=0 ... ');
try
    s0 = 15.0; sf = 85.0; t0 = 1.0; tf = 4.0;
    [s_bnd, ds_bnd, dds_bnd] = quintic_trajectory(s0, sf, t0, tf, [t0, tf]);
    assert(abs(s_bnd(1) - s0) < 1e-12, 'Initial position boundary mismatch.');
    assert(abs(s_bnd(2) - sf) < 1e-12, 'Final position boundary mismatch.');
    assert(abs(ds_bnd(1)) < 1e-12, 'Initial velocity is non-zero.');
    assert(abs(ds_bnd(2)) < 1e-12, 'Final velocity is non-zero.');
    assert(abs(dds_bnd(1)) < 1e-12, 'Initial acceleration is non-zero.');
    assert(abs(dds_bnd(2)) < 1e-12, 'Final acceleration is non-zero.');

    fprintf('PASS (error < 1e-12)\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 3: Quintic Polynomial C^2 Continuity Verification
fprintf('[TEST 03] Quintic polynomial internal C^2 continuity (d(ds)/dt == dds) ... ');
try
    t_fine = linspace(0, 2.0, 500);
    [~, ds_fine, dds_fine] = quintic_trajectory(0.0, 100.0, 0.0, 2.0, t_fine);
    dt_val = t_fine(2) - t_fine(1);
    num_acc = diff(ds_fine) / dt_val;
    acc_mid = (dds_fine(1:end-1) + dds_fine(2:end)) / 2;
    err_acc = max(abs(num_acc - acc_mid));
    assert(err_acc < 0.1, sprintf('Acceleration continuity error too high: %.4f', err_acc));

    fprintf('PASS (max num diff = %.2e)\n', err_acc);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 4: Quintic Polynomial Error Handling on Invalid Intervals
fprintf('[TEST 04] Quintic error handling on invalid time duration (tf <= t0) ... ');
try
    caught_err = false;
    try
        quintic_trajectory(0, 10, 3.0, 1.0, 2.0);
    catch
        caught_err = true;
    end
    assert(caught_err, 'Failed to throw error when tf < t0.');
    fprintf('PASS\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 5: Pallet Grid Generator Dimensions & Spacing Offsets (2x2x1)
fprintf('[TEST 05] Pallet grid generator 2x2x1 dimensions and exact spacing offsets ... ');
try
    origin = [140.0, 120.0, 20.0];
    spacing = [35.0, 35.0, 25.0];
    pts = pallet_grid_generator(origin, 2, 2, 1, spacing);
    expected = [ ...
        140.0, 120.0, 20.0;
        140.0, 155.0, 20.0;
        175.0, 120.0, 20.0;
        175.0, 155.0, 20.0
    ];
    err_pts = norm(pts - expected, 'fro');
    assert(err_pts < 1e-12, 'Grid coordinates offset mismatch.');
    assert(isequal(size(pts), [4, 3]), 'Grid size must be 4x3.');

    fprintf('PASS (Frobenius err = %.2e)\n', err_pts);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 6: Pallet Grid Multi-Layer Vertical Stacking (2x2x3)
fprintf('[TEST 06] Pallet grid 2x2x3 multi-layer vertical stacking coordinates (12 points) ... ');
try
    origin = [140.0, 120.0, 20.0];
    spacing = [35.0, 35.0, 25.0];
    [pts_3d, is_valid_grid, details] = pallet_grid_generator(origin, 2, 2, 3, spacing);
    assert(size(pts_3d, 1) == 12, 'Point count must be 12 for 2x2x3.');
    assert(min(pts_3d(:, 3)) == 20.0, 'Base layer elevation must be 20.0 mm.');
    assert(max(pts_3d(:, 3)) == 70.0, 'Top layer elevation must be 70.0 mm.');
    assert(is_valid_grid == true, 'Valid pallet grid flagged as invalid.');

    fprintf('PASS (12 points, Z in [20, 70] mm)\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 7: Pallet Grid Workspace Compliance (R <= 260 mm)
fprintf('[TEST 07] Pallet grid coordinates reachability compliance (R <= 260 mm) ... ');
try
    origin = [140.0, 120.0, 20.0];
    spacing = [35.0, 35.0, 25.0];
    [pts_3d, is_valid_grid] = pallet_grid_generator(origin, 2, 2, 3, spacing);
    radii = sqrt(pts_3d(:, 1).^2 + pts_3d(:, 2).^2);
    max_r = max(radii);
    assert(max_r <= 260.0, sprintf('Grid point exceeds reach limit: R=%.2f mm > 260 mm', max_r));
    assert(is_valid_grid, 'Pallet grid reachability check failed.');

    fprintf('PASS (max R = %.2f mm <= 260 mm)\n', max_r);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 8: Pallet Grid Non-Positive Spacing Rejection
fprintf('[TEST 08] Pallet grid non-positive spacing error validation ... ');
try
    caught_spacing_err = false;
    try
        pallet_grid_generator([140, 120, 20], 2, 2, 1, [-10, 0, 25]);
    catch
        caught_spacing_err = true;
    end
    assert(caught_spacing_err, 'Failed to detect invalid spacing <= 0.');
    fprintf('PASS\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 9: Multi-Point Planner 8-Step Pick-and-Place Trajectory
fprintf('[TEST 09] Multi-point planner 8-step pick-and-place trajectory generation ... ');
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0, 2.0; % P1: Home
        220.0, -100.0, 120.0, 0.0, 2.0; % P2: Pick Hover
        220.0, -100.0,  40.0, 0.0, 1.5; % P3: Pick Point
        220.0, -100.0, 120.0, 0.0, 1.5; % P4: Pick Retract
        180.0,  160.0, 140.0, 0.0, 2.5; % P5: Transit
        180.0,  160.0,  70.0, 0.0, 1.5; % P6: Place Point
        180.0,  160.0, 140.0, 0.0, 1.5; % P7: Place Retract
        240.0,    0.0, 180.0, 0.0, 2.0  % P8: Return Home
    ];
    traj = multi_point_planner(wp_8step, [], 0.02, 'cartesian');
    assert(traj.is_valid, 'Trajectory planner returned is_valid=false for reachable 8-step cycle.');
    assert(size(traj.theta, 2) == 4, 'Joint trajectory must have 4 columns.');
    assert(size(traj.pose, 2) == 4, 'Cartesian pose must have 4 columns.');
    assert(traj.t(end) >= 12.0, 'Total trajectory duration too short.');

    fprintf('PASS (Total duration = %.2f s, M = %d samples)\n', traj.total_duration, numel(traj.t));
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 10: Multi-Point Planner C^2 Boundary Continuity (Jump < 1e-3)
fprintf('[TEST 10] C^2 continuity across all segment boundaries (acceleration jump < 1e-3) ... ');
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0, 2.0;
        220.0, -100.0, 120.0, 0.0, 2.0;
        220.0, -100.0,  40.0, 0.0, 1.5;
        220.0, -100.0, 120.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 2.5;
        180.0,  160.0,  70.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 1.5;
        240.0,    0.0, 180.0, 0.0, 2.0
    ];
    traj = multi_point_planner(wp_8step, [], 0.01, 'cartesian');

    % Check segment boundaries
    seg_durs = traj.segment_durations;
    t_boundaries = cumsum(seg_durs);
    max_acc_jump = 0.0;
    max_vel_jump = 0.0;

    for k = 1:(numel(t_boundaries) - 1)
        tb = t_boundaries(k);
        % Find samples immediately before and after boundary
        idx_before = find(traj.t <= tb, 1, 'last');
        idx_after  = find(traj.t >= tb, 1, 'first');

        if idx_before > 0 && idx_after <= numel(traj.t)
            v_jump = max(abs(traj.dtheta(idx_after, :) - traj.dtheta(idx_before, :)));
            a_jump = max(abs(traj.ddtheta(idx_after, :) - traj.ddtheta(idx_before, :)));
            max_vel_jump = max(max_vel_jump, v_jump);
            max_acc_jump = max(max_acc_jump, a_jump);
        end
    end

    assert(max_acc_jump < 1e-3, sprintf('Max acceleration jump discontinuity = %.2e >= 1e-3', max_acc_jump));
    assert(max_vel_jump < 1e-3, sprintf('Max velocity jump discontinuity = %.2e >= 1e-3', max_vel_jump));

    fprintf('PASS (max dtheta jump = %.2e, max ddtheta jump = %.2e < 1e-3)\n', max_vel_jump, max_acc_jump);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 11: Multi-Point Planner Joint Velocity Limits Compliance (<= 120 deg/s)
fprintf('[TEST 11] Joint velocity limits compliance (|dtheta| <= 120 deg/s = 2.0944 rad/s) ... ');
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0, 2.0;
        220.0, -100.0, 120.0, 0.0, 2.0;
        220.0, -100.0,  40.0, 0.0, 1.5;
        220.0, -100.0, 120.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 2.5;
        180.0,  160.0,  70.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 1.5;
        240.0,    0.0, 180.0, 0.0, 2.0
    ];
    traj = multi_point_planner(wp_8step, [], 0.02, 'cartesian');
    MAX_VEL_RAD = deg2rad(120.0); % 2.094395 rad/s
    peak_vel = max(abs(traj.dtheta(:)));
    assert(peak_vel <= MAX_VEL_RAD + 1e-4, ...
           sprintf('Peak joint velocity exceeded: %.4f rad/s > %.4f rad/s', peak_vel, MAX_VEL_RAD));

    fprintf('PASS (peak joint velocity = %.4f rad/s <= %.4f rad/s)\n', peak_vel, MAX_VEL_RAD);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 12: Simulation Trajectory Streamer Data Format & Monotonicity
fprintf('[TEST 12] Simulation trajectory streamer structure and time monotonicity ... ');
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0, 2.0;
        220.0, -100.0, 120.0, 0.0, 2.0;
        220.0, -100.0,  40.0, 0.0, 1.5;
        240.0,    0.0, 180.0, 0.0, 2.0
    ];
    traj = multi_point_planner(wp_8step, [], 0.02, 'cartesian');
    [sim_results, stream_data] = simulate_trajectory(traj);

    assert(isfield(stream_data, 'time'), 'Missing time field in stream_data.');
    assert(isfield(stream_data, 'signals'), 'Missing signals field in stream_data.');
    assert(stream_data.signals.dimensions == 4, 'signals.dimensions must equal 4.');
    assert(all(diff(stream_data.time) > 0), 'Time vector is not strictly monotonic.');

    fprintf('PASS (stream_data verified)\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 13: Simulation Trajectory Tracking Error Accuracy (< 1.0 mm)
fprintf('[TEST 13] Trajectory tracking error accuracy (< 1.0 mm) ... ');
try
    wp_8step = [ ...
        240.0,    0.0, 180.0, 0.0, 2.0;
        220.0, -100.0, 120.0, 0.0, 2.0;
        220.0, -100.0,  40.0, 0.0, 1.5;
        220.0, -100.0, 120.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 2.5;
        180.0,  160.0,  70.0, 0.0, 1.5;
        180.0,  160.0, 140.0, 0.0, 1.5;
        240.0,    0.0, 180.0, 0.0, 2.0
    ];
    traj = multi_point_planner(wp_8step, [], 0.02, 'cartesian');
    % Simulate with small sensor noise (0.0001 rad = 0.1 mrad)
    sim_opt.noise_std = 0.0001;
    [sim_res, ~] = simulate_trajectory(traj, sim_opt);

    assert(sim_res.is_tracking_compliant, ...
           sprintf('Tracking compliance failed: max error = %.4f mm >= 1.0 mm', sim_res.max_cartesian_error));
    assert(sim_res.max_cartesian_error < 1.0, ...
           sprintf('Max tracking error = %.4f mm >= 1.0 mm', sim_res.max_cartesian_error));

    fprintf('PASS (max error = %.4f mm < 1.0 mm, mean = %.4f mm)\n', ...
            sim_res.max_cartesian_error, sim_res.mean_cartesian_error);
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 14: Zero Peter Corke Robotics Toolbox Dependency Scan
fprintf('[TEST 14] Zero Peter Corke Robotics Toolbox dependency verification ... ');
try
    corke_signatures = { ...
        'SerialLink', 'Link(', 'Revolute(', 'Prismatic(', ...
        'fkine(', 'ikine(', 'ikunc(', 'jacob0(', 'jacobn(', ...
        'tr2rpy(', 'rpy2tr(', 'transl(', 'trotx(', 'troty(', 'trotz(', ...
        'ctraj(', 'jtraj(', 'qplot(', 'teach(' ...
    };
    trajectory_files = dir(fullfile(current_dir, '*.m'));
    has_violation = false;
    violating_file = '';

    for f = 1:numel(trajectory_files)
        if strcmp(trajectory_files(f).name, 'test_trajectory_planning.m')
            continue;
        end
        filepath = fullfile(current_dir, trajectory_files(f).name);
        fid = fopen(filepath, 'r');
        if fid < 0, continue; end
        while ~feof(fid)
            line = strtrim(fgetl(fid));
            if ischar(line) && ~isempty(line) && ~startsWith(line, '%')
                for s = 1:numel(corke_signatures)
                    if contains(line, corke_signatures{s})
                        has_violation = true;
                        violating_file = trajectory_files(f).name;
                        break;
                    end
                end
            end
            if has_violation, break; end
        end
        fclose(fid);
        if has_violation, break; end
    end

    assert(~has_violation, sprintf('Corke toolbox signature found in %s', violating_file));
    fprintf('PASS (All %d trajectory files clean)\n', numel(trajectory_files));
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Test 15: Graceful Handling of Unreachable Cartesian Waypoints
fprintf('[TEST 15] Graceful handling of unreachable Cartesian waypoints (is_valid=false) ... ');
try
    % Waypoint 1 reachable, Waypoint 2 unreachable (R = 600 mm, exceeds R_max = 260 mm)
    unreachable_wps = [200.0, 0.0, 100.0, 0.0; ...
                       600.0, 0.0, 100.0, 0.0];
    traj_unreach = multi_point_planner(unreachable_wps);
    assert(~traj_unreach.is_valid, 'Trajectory should be marked invalid for unreachable waypoint.');
    assert(isfield(traj_unreach, 'error_msg') && ~isempty(traj_unreach.error_msg), ...
           'Trajectory should contain a descriptive error_msg.');
    assert(contains(traj_unreach.error_msg, 'unreachable'), ...
           sprintf('error_msg does not mention unreachable: %s', traj_unreach.error_msg));

    fprintf('PASS (is_valid=false handled gracefully)\n');
    tests_passed = tests_passed + 1;
catch ME
    fprintf('FAIL (%s)\n', ME.message);
    tests_failed = tests_failed + 1;
end

% Summary
fprintf('\n================================================================================\n');
fprintf('  TEST SUMMARY: %d / %d PASSED (%.1f%%)\n', tests_passed, total_tests, (tests_passed/total_tests)*100);
if tests_failed == 0
    fprintf('  STATUS: >>> ALL MILESTONE 3 TESTS PASSED SUCCESSFULLY <<<\n');
else
    fprintf('  STATUS: >>> %d TEST(S) FAILED <<<\n', tests_failed);
end
fprintf('================================================================================\n');

if tests_failed > 0
    error('test_trajectory_planning:FailedTests', '%d of %d tests failed.', tests_failed, total_tests);
end
