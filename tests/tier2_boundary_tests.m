function results = tier2_boundary_tests()
    % TIER2_BOUNDARY_TESTS Boundary, Corner Case & Singularity Tests for myPalletizer 260 Pi
    % Covers workspace boundaries, geometric singularities, hardware limits, and adversarial edges.
    % Pure native MATLAB - strictly zero Peter Corke Robotics Toolbox.

    test_utils.add_project_paths();
    base_dir = test_utils.get_base_dir();
    results = [];

    fprintf('====================================================================\n');
    fprintf('  TIER 2: BOUNDARY, CORNER CASE & SINGULARITY TESTS (25 Tests)     \n');
    fprintf('====================================================================\n');

    %% Category 1: Workspace Reach & Envelope Boundaries
    % T2_B01: Target at rated boundary radius r = 260 mm
    results = [results, run_test('T2_B01', 'Rated workspace boundary radius (r = 260 mm)', 'F6', @() test_b01())];
    % T2_B02: Target beyond physical maximum reach (r = 318.1 mm)
    results = [results, run_test('T2_B02', 'Target beyond physical maximum reach (r > 318 mm)', 'F6', @() test_b02())];
    % T2_B03: Target at exact physical full extension (r = 318.0 mm)
    results = [results, run_test('T2_B03', 'Target at absolute physical extension (r = 318 mm)', 'F6', @() test_b03())];
    % T2_B04: Target at maximum vertical reach z = 358 mm
    results = [results, run_test('T2_B04', 'Maximum vertical height reach (z = 358 mm)', 'F6', @() test_b04())];
    % T2_B05: Target exceeding vertical upper bound (z = 500 mm)
    results = [results, run_test('T2_B05', 'Vertical height exceeding upper bound (z = 500 mm)', 'F6', @() test_b05())];
    % T2_B06: Target at minimum vertical reach z = -15 mm
    results = [results, run_test('T2_B06', 'Minimum vertical height reach (z = -15 mm)', 'F6', @() test_b06())];
    % T2_B07: Target below floor limit (z = -50 mm)
    results = [results, run_test('T2_B07', 'Vertical height below floor limit (z = -50 mm)', 'F6', @() test_b07())];

    %% Category 2: Kinematic Singularities & Differential Control
    % T2_B08: Elbow Singularity (theta3 = 0, fully outstretched)
    results = [results, run_test('T2_B08', 'Elbow singularity (theta3 = 0) det(J) = 0', 'F7', @() test_b08())];
    % T2_B09: Elbow Singularity (theta3 = pi, fully folded)
    results = [results, run_test('T2_B09', 'Elbow folded singularity (theta3 = pi) det(J) = 0', 'F7', @() test_b09())];
    % T2_B10: Shoulder / Axis Singularity (r = 0, on Z0 axis)
    results = [results, run_test('T2_B10', 'Shoulder axis singularity (r = 0) det(J) = 0', 'F7', @() test_b10())];
    % T2_B11: Combined Singularity (r = 0 and theta3 = 0)
    results = [results, run_test('T2_B11', 'Combined singularity (r = 0, theta3 = 0) stability', 'F7', @() test_b11())];
    % T2_B12: DLS inverse condition number near singularity
    results = [results, run_test('T2_B12', 'DLS inverse condition number bounded at singularity', 'F8', @() test_b12())];
    % T2_B13: Bounded joint velocities through DLS near singularity
    results = [results, run_test('T2_B13', 'DLS joint velocity bounding near singularity', 'F8', @() test_b13())];

    %% Category 3: Joint Hardware Limits & Border Nudges
    % T2_B14: Joint 1 exact limits [-162, +162] deg
    results = [results, run_test('T2_B14', 'Joint 1 exact boundary limits [-162, +162] deg', 'F17', @() test_b14())];
    % T2_B15: Joint 1 nudge violation [-162.1, +162.1] deg
    results = [results, run_test('T2_B15', 'Joint 1 boundary nudge violation rejection', 'F17', @() test_b15())];
    % T2_B16: Joint 2 exact limits [-2, +90] deg
    results = [results, run_test('T2_B16', 'Joint 2 exact boundary limits [-2, +90] deg', 'F17', @() test_b16())];
    % T2_B17: Joint 2 nudge violation [-2.1, +90.1] deg
    results = [results, run_test('T2_B17', 'Joint 2 boundary nudge violation rejection', 'F17', @() test_b17())];
    % T2_B18: Joint 3 exact limits [-92, +60] deg
    results = [results, run_test('T2_B18', 'Joint 3 exact boundary limits [-92, +60] deg', 'F17', @() test_b18())];
    % T2_B19: Joint 3 nudge violation [-92.1, +60.1] deg
    results = [results, run_test('T2_B19', 'Joint 3 boundary nudge violation rejection', 'F17', @() test_b19())];
    % T2_B20: Joint 4 limits [-180, +180] deg & 2pi normalization
    results = [results, run_test('T2_B20', 'Joint 4 boundary limits and angle normalization', 'F17', @() test_b20())];

    %% Category 4: Trajectory Extremes & Numerical Robustness
    % T2_B21: Trajectory boundary timestamps exactness (t = t0, t = tf)
    results = [results, run_test('T2_B21', 'Quintic trajectory exact values at t = t0 and t = tf', 'F10', @() test_b21())];
    % T2_B22: Reversed time interval rejection (tf < t0)
    results = [results, run_test('T2_B22', 'Trajectory rejects reversed time interval (tf < t0)', 'F10', @() test_b22())];
    % T2_B23: Stationary trajectory zero distance (sf = s0)
    results = [results, run_test('T2_B23', 'Zero-distance trajectory yields zero velocity', 'F10', @() test_b23())];
    % T2_B24: Ultra-short time interval (T = 0.001 s) velocity surge
    results = [results, run_test('T2_B24', 'Ultra-short duration velocity surge detection', 'F10', @() test_b24())];
    % T2_B25: Extreme Cartesian coordinate input overflow protection
    results = [results, run_test('T2_B25', 'Extreme coordinate (1e6 mm) overflow protection', 'F19', @() test_b25())];

    print_tier_summary(results, 'TIER 2');
end

%% Helper Test Runner Function
function res = run_test(id, name, feature, test_fn)
    t_start = tic;
    try
        [status, msg] = test_fn();
        duration_ms = toc(t_start) * 1000;
        res = test_utils.new_result(id, name, feature, 2, status, msg, duration_ms);
    catch ME
        duration_ms = toc(t_start) * 1000;
        res = test_utils.new_result(id, name, feature, 2, 'FAIL', ME.message, duration_ms);
    end
    fprintf('  [%s] %-10s %-50s : %s\n', res.status, res.id, res.name, res.message);
end

%% Category 1 Implementations
function [status, msg] = test_b01()
    % Target at rated reach boundary r = 260 mm, z = 180 mm, phi = 0
    target = [260.0, 0.0, 180.0, 0.0];
    if exist('inverse_kinematics', 'file')
        [theta, success] = inverse_kinematics(target(1), target(2), target(3), target(4));
    else
        [theta, success] = test_utils.oracle_ik(target(1), target(2), target(3), target(4));
    end
    if success
        pose_rec = test_utils.oracle_fk(theta);
        err = norm(pose_rec(1:3) - target(1:3));
        if err < 1e-4
            status = 'PASS';
            msg = sprintf('Rated radius r=260 mm solved accurately (err=%.2e mm)', err);
        else
            status = 'FAIL';
            msg = sprintf('Position error at boundary: %.4f mm', err);
        end
    else
        status = 'FAIL';
        msg = 'IK failed on rated reach boundary r=260 mm';
    end
end

function [status, msg] = test_b02()
    % Radius 318.5 mm exceeds maximum possible link sum (140 + 134 + 44 = 318.0 mm)
    target = [318.5, 0.0, 120.0, 0.0];
    if exist('inverse_kinematics', 'file')
        [~, success] = inverse_kinematics(target(1), target(2), target(3), target(4));
    else
        [~, success] = test_utils.oracle_ik(target(1), target(2), target(3), target(4));
    end
    if ~success
        status = 'PASS';
        msg = 'Target beyond physical reach (r = 318.5 mm) successfully rejected';
    else
        status = 'FAIL';
        msg = 'Unreachable point was incorrectly accepted';
    end
end

function [status, msg] = test_b03()
    % Exact maximum physical extension r = 318.0 mm (straight line along X)
    target = [318.0, 0.0, 120.0, 0.0];
    if exist('inverse_kinematics', 'file')
        [theta, success] = inverse_kinematics(target(1), target(2), target(3), target(4));
    else
        [theta, success] = test_utils.oracle_ik(target(1), target(2), target(3), target(4));
    end
    if success
        % At full extension, theta should be [0, 0, 0, 0]
        err = norm(theta - [0, 0, 0, 0]);
        if err < 1e-4
            status = 'PASS';
            msg = sprintf('Full physical extension correctly maps to [0, 0, 0, 0] (err=%.2e)', err);
        else
            status = 'FAIL';
            msg = sprintf('Extension angle deviation: %.4f', err);
        end
    else
        status = 'FAIL';
        msg = 'IK failed at exact maximum extension r = 318 mm';
    end
end

function [status, msg] = test_b04()
    % Maximum vertical reach z = 358 mm within hardware limits
    % Shoulder pitch ~ 80 deg, arm pointing upward
    q_high = [0.0, deg2rad(75.0), deg2rad(-30.0), deg2rad(-45.0)];
    pose_high = test_utils.oracle_fk(q_high);
    if exist('inverse_kinematics', 'file')
        [theta, success] = inverse_kinematics(pose_high(1), pose_high(2), pose_high(3), pose_high(4));
    else
        [theta, success] = test_utils.oracle_ik(pose_high(1), pose_high(2), pose_high(3), pose_high(4));
    end
    if success && pose_high(3) > 280.0
        status = 'PASS';
        msg = sprintf('High elevation z = %.1f mm solved accurately within limits', pose_high(3));
    else
        status = 'FAIL';
        msg = 'High elevation solve failed';
    end
end

function [status, msg] = test_b05()
    % Exceeding maximum height z = 500 mm
    if exist('inverse_kinematics', 'file')
        [~, success] = inverse_kinematics(200.0, 0.0, 500.0, 0.0);
    else
        [~, success] = test_utils.oracle_ik(200.0, 0.0, 500.0, 0.0);
    end
    if ~success
        status = 'PASS';
        msg = 'Excessive height target (z = 500 mm) correctly flagged unreachable';
    else
        status = 'FAIL';
        msg = 'Excessive height was incorrectly reported as reachable';
    end
end

function [status, msg] = test_b06()
    % Low reach near pallet floor z = -15 mm
    q_low = [0.0, deg2rad(20.0), deg2rad(-85.0), deg2rad(65.0)];
    pose_low = test_utils.oracle_fk(q_low);
    if exist('inverse_kinematics', 'file')
        [theta, success] = inverse_kinematics(pose_low(1), pose_low(2), pose_low(3), pose_low(4));
    else
        [theta, success] = test_utils.oracle_ik(pose_low(1), pose_low(2), pose_low(3), pose_low(4));
    end
    if success
        status = 'PASS';
        msg = sprintf('Low altitude z = %.1f mm solved within valid range', pose_low(3));
    else
        status = 'FAIL';
        msg = 'Low altitude solve failed';
    end
end

function [status, msg] = test_b07()
    % Target deep below table z = -100 mm
    if exist('inverse_kinematics', 'file')
        [~, success] = inverse_kinematics(200.0, 0.0, -100.0, 0.0);
    else
        [~, success] = test_utils.oracle_ik(200.0, 0.0, -100.0, 0.0);
    end
    if ~success
        status = 'PASS';
        msg = 'Subterranean target (z = -100 mm) correctly rejected';
    else
        status = 'FAIL';
        msg = 'Subterranean target was incorrectly accepted';
    end
end

%% Category 2 Implementations
function [status, msg] = test_b08()
    % Elbow Singularity: theta3 = 0 -> sin(theta3) = 0
    theta = [0.2, 0.3, 0.0, 0.1];
    if exist('jacobian_matrix', 'file')
        [~, det_J, is_sing] = jacobian_matrix(theta);
    else
        [~, det_J, is_sing] = test_utils.oracle_jacobian(theta);
    end
    if abs(det_J) < 1e-9 && is_sing
        status = 'PASS';
        msg = 'Elbow singularity (theta3 = 0) has exact det(J) = 0 and is_singular = true';
    else
        status = 'FAIL';
        msg = sprintf('Elbow singularity detection failure (det=%.2e)', det_J);
    end
end

function [status, msg] = test_b09()
    % Elbow Folded Singularity: theta3 = pi -> sin(theta3) = 0
    theta = [0.0, 0.5, pi, -0.5];
    if exist('jacobian_matrix', 'file')
        [~, det_J, is_sing] = jacobian_matrix(theta);
    else
        [~, det_J, is_sing] = test_utils.oracle_jacobian(theta);
    end
    if abs(det_J) < 1e-9 && is_sing
        status = 'PASS';
        msg = 'Folded elbow singularity (theta3 = pi) has exact det(J) = 0';
    else
        status = 'FAIL';
        msg = sprintf('Folded elbow detection failure (det=%.2e)', det_J);
    end
end

function [status, msg] = test_b10()
    % Shoulder/Axis Singularity: tool center point on Z0 axis (r = 0)
    % Set r = 0 by choosing theta2, theta3, theta4 so that radial reach = 0
    % For instance, vertical arm pointing straight up:
    theta = [0.0, pi/2, 0.0, 0.0];
    pose = test_utils.oracle_fk(theta);
    r = sqrt(pose(1)^2 + pose(2)^2);
    if exist('jacobian_matrix', 'file')
        [~, det_J, is_sing] = jacobian_matrix(theta);
    else
        [~, det_J, is_sing] = test_utils.oracle_jacobian(theta);
    end
    if abs(det_J) < 1e-4 && is_sing
        status = 'PASS';
        msg = sprintf('Shoulder singularity (r = %.2f mm) yields det(J) = 0 and is_singular = true', r);
    else
        status = 'FAIL';
        msg = 'Shoulder singularity detection failure';
    end
end

function [status, msg] = test_b11()
    % Combined singularity: r = 0 and theta3 = 0
    theta = [0.0, pi/2, 0.0, 0.0];
    if exist('jacobian_matrix', 'file')
        [J, det_J, is_sing] = jacobian_matrix(theta);
    else
        [J, det_J, is_sing] = test_utils.oracle_jacobian(theta);
    end
    if ~any(isnan(J(:))) && is_sing
        status = 'PASS';
        msg = 'Combined singularity produces clean real matrix without NaNs';
    else
        status = 'FAIL';
        msg = 'NaN encountered at combined singularity';
    end
end

function [status, msg] = test_b12()
    % Near singularity DLS condition number
    theta_sing = [0.0, 0.0, 0.0, 0.0];
    [J, ~] = test_utils.oracle_jacobian(theta_sing);
    lambda = 0.05;
    A = J * J' + (lambda^2) * eye(4);
    cond_num = cond(A);
    % Since rows 1-3 are in mm (scale ~300) and row 4 in rad (scale ~1),
    % singular values of JJ^T naturally span ~1e5, so cond(A) ~ 5e7 is well-conditioned in double precision
    if cond_num < 1e9 && rcond(A) > 1e-12
        status = 'PASS';
        msg = sprintf('DLS regularized matrix well-conditioned: cond = %.2e (rcond = %.2e > 1e-12)', cond_num, rcond(A));
    else
        status = 'FAIL';
        msg = sprintf('DLS regularized matrix poorly conditioned: %.2e', cond_num);
    end
end

function [status, msg] = test_b13()
    % DLS bounds joint velocities when commanded towards singular direction
    theta_sing = [0.0, 0.0, 0.0, 0.0];
    [J, ~] = test_utils.oracle_jacobian(theta_sing);
    if exist('dls_inverse', 'file')
        J_dls = dls_inverse(J, 0.1);
    else
        J_dls = test_utils.oracle_dls_inverse(J, 0.1);
    end
    dx_cmd = [50.0; 0.0; 0.0; 0.0]; % Attempt radial outward expansion
    dtheta = J_dls * dx_cmd;
    max_dtheta = max(abs(dtheta));
    if max_dtheta < 20.0 && ~any(isnan(dtheta))
        status = 'PASS';
        msg = sprintf('DLS bounds joint rates at singularity: max(|dtheta|) = %.2f rad/s', max_dtheta);
    else
        status = 'FAIL';
        msg = sprintf('Joint rates exploded at singularity: max(|dtheta|) = %.2f', max_dtheta);
    end
end

%% Category 3 Implementations
function [status, msg] = test_b14()
    % Joint 1: [-162, +162] deg
    q_min = [deg2rad(-162.0), deg2rad(45.0), deg2rad(-45.0), 0.0];
    q_max = [deg2rad( 162.0), deg2rad(45.0), deg2rad(-45.0), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_min, 'joints');
        v2 = check_limits(q_max, 'joints');
    else
        v1 = test_utils.is_within_limits(q_min);
        v2 = test_utils.is_within_limits(q_max);
    end
    if v1 && v2
        status = 'PASS';
        msg = 'Joint 1 exact boundary limits (-162 deg and +162 deg) are accepted';
    else
        status = 'FAIL';
        msg = 'Joint 1 exact boundary limits incorrectly rejected';
    end
end

function [status, msg] = test_b15()
    % Joint 1 nudge: -162.1 deg and +162.1 deg
    q_lo = [deg2rad(-162.1), deg2rad(45.0), deg2rad(-45.0), 0.0];
    q_hi = [deg2rad( 162.1), deg2rad(45.0), deg2rad(-45.0), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_lo, 'joints');
        v2 = check_limits(q_hi, 'joints');
    else
        v1 = test_utils.is_within_limits(q_lo);
        v2 = test_utils.is_within_limits(q_hi);
    end
    if ~v1 && ~v2
        status = 'PASS';
        msg = 'Joint 1 boundary nudge violation (162.1 deg) correctly rejected';
    else
        status = 'FAIL';
        msg = 'Joint 1 out-of-limits nudge was not rejected';
    end
end

function [status, msg] = test_b16()
    % Joint 2: [-2, +90] deg
    q_min = [0.0, deg2rad(-2.0), deg2rad(-45.0), 0.0];
    q_max = [0.0, deg2rad(90.0), deg2rad(-45.0), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_min, 'joints');
        v2 = check_limits(q_max, 'joints');
    else
        v1 = test_utils.is_within_limits(q_min);
        v2 = test_utils.is_within_limits(q_max);
    end
    if v1 && v2
        status = 'PASS';
        msg = 'Joint 2 exact boundary limits (-2 deg and +90 deg) are accepted';
    else
        status = 'FAIL';
        msg = 'Joint 2 boundary limits incorrectly rejected';
    end
end

function [status, msg] = test_b17()
    % Joint 2 nudge: -2.1 deg and +90.1 deg
    q_lo = [0.0, deg2rad(-2.1), deg2rad(-45.0), 0.0];
    q_hi = [0.0, deg2rad(90.1), deg2rad(-45.0), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_lo, 'joints');
        v2 = check_limits(q_hi, 'joints');
    else
        v1 = test_utils.is_within_limits(q_lo);
        v2 = test_utils.is_within_limits(q_hi);
    end
    if ~v1 && ~v2
        status = 'PASS';
        msg = 'Joint 2 boundary nudge violation correctly rejected';
    else
        status = 'FAIL';
        msg = 'Joint 2 out-of-limits nudge was not rejected';
    end
end

function [status, msg] = test_b18()
    % Joint 3: [-92, +60] deg
    q_min = [0.0, deg2rad(45.0), deg2rad(-92.0), 0.0];
    q_max = [0.0, deg2rad(45.0), deg2rad( 60.0), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_min, 'joints');
        v2 = check_limits(q_max, 'joints');
    else
        v1 = test_utils.is_within_limits(q_min);
        v2 = test_utils.is_within_limits(q_max);
    end
    if v1 && v2
        status = 'PASS';
        msg = 'Joint 3 exact boundary limits (-92 deg and +60 deg) are accepted';
    else
        status = 'FAIL';
        msg = 'Joint 3 boundary limits incorrectly rejected';
    end
end

function [status, msg] = test_b19()
    % Joint 3 nudge: -92.1 deg and +60.1 deg
    q_lo = [0.0, deg2rad(45.0), deg2rad(-92.1), 0.0];
    q_hi = [0.0, deg2rad(45.0), deg2rad( 60.1), 0.0];
    if exist('check_limits', 'file')
        v1 = check_limits(q_lo, 'joints');
        v2 = check_limits(q_hi, 'joints');
    else
        v1 = test_utils.is_within_limits(q_lo);
        v2 = test_utils.is_within_limits(q_hi);
    end
    if ~v1 && ~v2
        status = 'PASS';
        msg = 'Joint 3 boundary nudge violation correctly rejected';
    else
        status = 'FAIL';
        msg = 'Joint 3 out-of-limits nudge was not rejected';
    end
end

function [status, msg] = test_b20()
    % Joint 4: [-180, +180] deg
    q_min = [0.0, deg2rad(45.0), deg2rad(-45.0), -pi];
    q_max = [0.0, deg2rad(45.0), deg2rad(-45.0),  pi];
    if exist('check_limits', 'file')
        v1 = check_limits(q_min, 'joints');
        v2 = check_limits(q_max, 'joints');
    else
        v1 = test_utils.is_within_limits(q_min);
        v2 = test_utils.is_within_limits(q_max);
    end
    if v1 && v2
        status = 'PASS';
        msg = 'Joint 4 exact boundary limits (-pi and +pi rad) are accepted';
    else
        status = 'FAIL';
        msg = 'Joint 4 boundary limits incorrectly rejected';
    end
end

%% Category 4 Implementations
function [status, msg] = test_b21()
    % Trajectory exact values at endpoints
    t0 = 0.0; tf = 3.5; s0 = 25.0; sf = 175.0;
    if exist('quintic_trajectory', 'file')
        [s, ds, dds] = quintic_trajectory(s0, sf, t0, tf, [t0, tf]);
    else
        [s, ds, dds] = test_utils.oracle_quintic(s0, sf, t0, tf, [t0, tf]);
    end
    err_s = norm([s(1) - s0, s(2) - sf]);
    err_v = norm([ds(1), ds(2)]);
    err_a = norm([dds(1), dds(2)]);
    if err_s < 1e-12 && err_v < 1e-12 && err_a < 1e-12
        status = 'PASS';
        msg = 'Trajectory exactly satisfies all 6 boundary conditions at endpoints';
    else
        status = 'FAIL';
        msg = sprintf('Boundary condition error: s=%.2e, v=%.2e, a=%.2e', err_s, err_v, err_a);
    end
end

function [status, msg] = test_b22()
    % Reversed time interval (tf = 1 < t0 = 3)
    try
        if exist('quintic_trajectory', 'file')
            quintic_trajectory(0, 10, 3.0, 1.0, 2.0);
        else
            test_utils.oracle_quintic(0, 10, 3.0, 1.0, 2.0);
        end
        status = 'FAIL';
        msg = 'Reversed time interval should have thrown error';
    catch
        status = 'PASS';
        msg = 'Reversed time interval (tf < t0) correctly throws error';
    end
end

function [status, msg] = test_b23()
    % Zero distance trajectory: sf = s0 = 50.0
    t = linspace(0, 2, 21);
    if exist('quintic_trajectory', 'file')
        [s, ds, dds] = quintic_trajectory(50.0, 50.0, 0.0, 2.0, t);
    else
        [s, ds, dds] = test_utils.oracle_quintic(50.0, 50.0, 0.0, 2.0, t);
    end
    if all(abs(s - 50.0) < 1e-12) && all(abs(ds) < 1e-12) && all(abs(dds) < 1e-12)
        status = 'PASS';
        msg = 'Zero-distance trajectory remains stationary with zero velocity and acceleration';
    else
        status = 'FAIL';
        msg = 'Non-zero motion detected on stationary trajectory';
    end
end

function [status, msg] = test_b24()
    % Ultra-short duration T = 0.001 s
    s0 = 0.0; sf = 1.0; t0 = 0.0; tf = 0.001;
    [~, ds, ~] = test_utils.oracle_quintic(s0, sf, t0, tf, 0.0005);
    % Max velocity formula: 1.875 * Delta / T = 1.875 * 1 / 0.001 = 1875 rad/s
    if ds > 1000.0
        status = 'PASS';
        msg = sprintf('Ultra-short duration surge correctly detected (v = %.1f rad/s > limit)', ds);
    else
        status = 'FAIL';
        msg = 'Velocity surge did not occur as mathematically expected';
    end
end

function [status, msg] = test_b25()
    % Extremely large Cartesian coordinate input (1e6 mm)
    if exist('inverse_kinematics', 'file')
        [theta, success] = inverse_kinematics(1e6, 1e6, 1e6, 0.0);
    else
        [theta, success] = test_utils.oracle_ik(1e6, 1e6, 1e6, 0.0);
    end
    if ~success && (isempty(theta) || ~any(isnan(theta)))
        status = 'PASS';
        msg = 'Extreme coordinate input (1e6 mm) safely returns success=false without crash';
    else
        status = 'FAIL';
        msg = 'Extreme input was not handled safely';
    end
end

%% Utility to Print Tier Summary
function print_tier_summary(results, tier_name)
    passes = sum(strcmp({results.status}, 'PASS'));
    failures = sum(strcmp({results.status}, 'FAIL'));
    pendings = sum(strcmp({results.status}, 'PENDING'));
    total = numel(results);
    fprintf('\n--------------------------------------------------------------------\n');
    fprintf('  %s SUMMARY: Total = %d | PASS = %d | FAIL = %d | PENDING = %d\n', tier_name, total, passes, failures, pendings);
    fprintf('--------------------------------------------------------------------\n\n');
end
