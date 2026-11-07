% TEST_CHALLENGER_M3_1 Empirical Adversarial Challenge for Multi-Segment Trajectory Planning
% Milestone 3: Trajectory Planning & Palletizing Motion Controller
% Challenger: teamwork_challenger (Challenger M3-1)
%
% Tests multi-segment via-point trajectories across 50 random paths with 4 to 10 waypoints each.
% Numerically evaluates left and right limits of velocity and acceleration at every junction.
% Verifies velocity jump < 1e-3 rad/s and acceleration jump < 1e-3 rad/s^2 across all junctions.
% Strictly zero Peter Corke Robotics Toolbox dependencies.

fprintf('================================================================================\n');
fprintf('  ADVERSARIAL VERIFICATION SUITE: Multi-Segment Trajectory C^2 Continuity       \n');
fprintf('  Challenger: teamwork_challenger (Challenger M3-1)                             \n');
fprintf('  Target: quintic_trajectory.m & multi_point_planner.m                          \n');
fprintf('================================================================================\n\n');

projectRoot = 'd:\Lidar\myPalletizer_BTL';
addpath(fullfile(projectRoot, 'kinematics'));
addpath(fullfile(projectRoot, 'trajectory'));
addpath(fullfile(projectRoot, 'tests'));

% Fix random seed for exact reproducibility
rng(20261107, 'twister');

num_paths = 50;
results = struct();
results.paths = cell(num_paths, 1);
results.max_vel_jumps = zeros(num_paths, 1);
results.max_acc_jumps = zeros(num_paths, 1);
results.max_pos_jumps = zeros(num_paths, 1);
results.max_peak_velocities = zeros(num_paths, 1);
results.total_junctions_tested = 0;
results.all_junction_vel_jumps = [];
results.all_junction_acc_jumps = [];
results.all_junction_pos_jumps = [];

vel_jump_threshold = 1.0e-3; % rad/s
acc_jump_threshold = 1.0e-3; % rad/s^2
max_allowable_joint_vel = deg2rad(120.0) + 1.0e-4; % 2.0944 rad/s

% Joint limits for random path generation
q_min = deg2rad([-160.0,   0.0, -90.0, -170.0]);
q_max = deg2rad([ 160.0,  85.0,  55.0,  170.0]);

fprintf('--- PART 1: STRESS-TESTING 50 ADVERSARIAL MULTI-SEGMENT PATHS ---\n\n');

total_failed_paths = 0;
path_categories = cell(num_paths, 1);

for p = 1:num_paths
    % Waypoint count: 4 to 10 waypoints
    num_wp = randi([4, 10]);
    
    % Diverse path categorization:
    % 1-15:  Random Joint space paths (uniform random in joint envelope)
    % 16-30: Cartesian reachable workspace paths (FK of legal configurations)
    % 31-40: High-speed stress paths (short durations 0.1s - 0.4s with large displacements)
    % 41-45: Zero-displacement and micro-motion paths (consecutive identical/sub-micron waypoints)
    % 46-50: Alternating oscillatory direction reversal paths
    
    if p <= 15
        cat_name = 'Random Joint Space';
        wp_joint = zeros(num_wp, 4);
        for i = 1:num_wp
            wp_joint(i, :) = q_min + rand(1, 4) .* (q_max - q_min);
        end
        wp_input = wp_joint;
        durations = 0.5 + 2.5 * rand(num_wp - 1, 1);
        wp_type = 'joint';
        
    elseif p <= 30
        cat_name = 'Cartesian Reachable';
        % Generate guaranteed reachable Cartesian points via FK of well-conditioned configurations
        wp_cart = zeros(num_wp, 4);
        for i = 1:num_wp
            q_c = [deg2rad(-150.0 + 300.0 * rand()), ...
                   deg2rad(  15.0 +  50.0 * rand()), ...
                   deg2rad( -60.0 +  50.0 * rand()), ...
                   deg2rad( -45.0 +  90.0 * rand())];
            wp_cart(i, :) = forward_kinematics(q_c);
        end
        wp_input = wp_cart;
        durations = 1.0 + 2.0 * rand(num_wp - 1, 1);
        wp_type = 'cartesian';
        
    elseif p <= 40
        cat_name = 'High-Speed Stress';
        wp_joint = zeros(num_wp, 4);
        for i = 1:num_wp
            wp_joint(i, :) = q_min + rand(1, 4) .* (q_max - q_min);
        end
        wp_input = wp_joint;
        % Extremely short durations to trigger auto-duration stretching
        durations = 0.1 + 0.3 * rand(num_wp - 1, 1);
        wp_type = 'joint';
        
    elseif p <= 45
        cat_name = 'Zero/Micro Displacement';
        base_q = q_min + 0.5 * (q_max - q_min);
        wp_joint = repmat(base_q, num_wp, 1);
        % Inject occasional tiny micro-perturbations (1e-6 rad) or identical points
        for i = 2:num_wp
            if mod(i, 2) == 0
                wp_joint(i, :) = base_q + 1e-6 * (rand(1, 4) - 0.5);
            end
        end
        wp_input = wp_joint;
        durations = 1.0 * ones(num_wp - 1, 1);
        wp_type = 'joint';
        
    else
        cat_name = 'Oscillatory Reversal';
        wp_joint = zeros(num_wp, 4);
        q_pos = deg2rad([ 120.0, 70.0,  40.0,  120.0]);
        q_neg = deg2rad([-120.0, 10.0, -70.0, -120.0]);
        for i = 1:num_wp
            if mod(i, 2) == 1
                wp_joint(i, :) = q_pos;
            else
                wp_joint(i, :) = q_neg;
            end
        end
        wp_input = wp_joint;
        durations = 0.8 + 1.2 * rand(num_wp - 1, 1);
        wp_type = 'joint';
    end
    
    path_categories{p} = cat_name;
    
    % Plan trajectory with multi_point_planner
    dt = 0.02;
    traj = multi_point_planner(wp_input, durations, dt, wp_type);
    
    % Number of segments = num_wp - 1
    % Number of junctions = num_wp - 2
    num_segs = num_wp - 1;
    num_junctions = num_segs - 1;
    
    seg_durs = traj.segment_durations;
    q_resolved = traj.waypoints_joint;
    
    max_path_vel_jump = 0.0;
    max_path_acc_jump = 0.0;
    max_path_pos_jump = 0.0;
    
    t_accum = 0.0;
    for seg = 1:num_segs
        t_accum = t_accum + seg_durs(seg);
        if seg < num_segs
            % Junction k is at time t_junction
            t_junction = t_accum;
            t_start_k = t_accum - seg_durs(seg);
            t_end_k   = t_accum;
            t_start_kp1 = t_accum;
            t_end_kp1   = t_accum + seg_durs(seg + 1);
            
            % 1. Analytical left limit from segment k at t = t_junction
            [q_left, v_left, a_left] = quintic_trajectory(...
                q_resolved(seg, :), q_resolved(seg + 1, :), ...
                t_start_k, t_end_k, t_junction);
            
            % 2. Analytical right limit from segment k+1 at t = t_junction
            [q_right, v_right, a_right] = quintic_trajectory(...
                q_resolved(seg + 1, :), q_resolved(seg + 2, :), ...
                t_start_kp1, t_end_kp1, t_junction);
            
            pos_jump = max(abs(q_right - q_left));
            vel_jump = max(abs(v_right - v_left));
            acc_jump = max(abs(a_right - a_left));
            
            % 3. Numerical epsilon-limits check as epsilon -> 0
            for eps_val = [1e-4, 1e-5, 1e-6]
                [~, v_l_eps, a_l_eps] = quintic_trajectory(...
                    q_resolved(seg, :), q_resolved(seg + 1, :), ...
                    t_start_k, t_end_k, t_junction - eps_val);
                [~, v_r_eps, a_r_eps] = quintic_trajectory(...
                    q_resolved(seg + 1, :), q_resolved(seg + 2, :), ...
                    t_start_kp1, t_end_kp1, t_junction + eps_val);
                eps_vel_jump = max(abs(v_r_eps - v_l_eps));
                eps_acc_jump = max(abs(a_r_eps - a_l_eps));
                
                % Numerical limits must converge toward 0
                if eps_val == 1e-6
                    assert(eps_vel_jump < vel_jump_threshold, ...
                        sprintf('Numerical epsilon velocity jump too large: %.2e', eps_vel_jump));
                    assert(eps_acc_jump < acc_jump_threshold, ...
                        sprintf('Numerical epsilon acceleration jump too large: %.2e', eps_acc_jump));
                end
            end
            
            max_path_pos_jump = max(max_path_pos_jump, pos_jump);
            max_path_vel_jump = max(max_path_vel_jump, vel_jump);
            max_path_acc_jump = max(max_path_acc_jump, acc_jump);
            
            results.total_junctions_tested = results.total_junctions_tested + 1;
            results.all_junction_pos_jumps(end + 1, 1) = pos_jump;
            results.all_junction_vel_jumps(end + 1, 1) = vel_jump;
            results.all_junction_acc_jumps(end + 1, 1) = acc_jump;
        end
    end
    
    results.max_pos_jumps(p) = max_path_pos_jump;
    results.max_vel_jumps(p) = max_path_vel_jump;
    results.max_acc_jumps(p) = max_path_acc_jump;
    results.max_peak_velocities(p) = traj.max_velocity;
    
    % Verification assertions for this path
    is_vel_ok = (max_path_vel_jump < vel_jump_threshold);
    is_acc_ok = (max_path_acc_jump < acc_jump_threshold);
    is_speed_limit_ok = (traj.max_velocity <= max_allowable_joint_vel);
    is_monotonic_ok = all(diff(traj.t) > 0);
    
    path_pass = is_vel_ok && is_acc_ok && is_speed_limit_ok && is_monotonic_ok;
    if ~path_pass
        total_failed_paths = total_failed_paths + 1;
        status_str = 'FAIL';
    else
        status_str = 'PASS';
    end
    
    fprintf('Path %02d/50 [%-25s | %2d WPs | %2d junc]: pos_j=%.1e rad, vel_j=%.1e rad/s, acc_j=%.1e rad/s^2, v_peak=%.3f rad/s -> %s\n', ...
        p, cat_name, num_wp, num_junctions, max_path_pos_jump, max_path_vel_jump, max_path_acc_jump, traj.max_velocity, status_str);
end

fprintf('\n================================================================================\n');
fprintf('                           ADVERSARIAL STRESS TEST SUMMARY                      \n');
fprintf('================================================================================\n');
fprintf('Total Paths Tested        : %d\n', num_paths);
fprintf('Total Junctions Evaluated : %d\n', results.total_junctions_tested);
fprintf('Max Position Jump         : %.4e rad (Exact zero: < 1e-12)\n', max(results.all_junction_pos_jumps));
fprintf('Max Velocity Jump         : %.4e rad/s (Threshold: < 1.0e-3 rad/s)\n', max(results.all_junction_vel_jumps));
fprintf('Max Acceleration Jump     : %.4e rad/s^2 (Threshold: < 1.0e-3 rad/s^2)\n', max(results.all_junction_acc_jumps));
fprintf('Peak Joint Velocity Across All Paths: %.4f rad/s (Limit: 2.0944 rad/s)\n', max(results.max_peak_velocities));
fprintf('Failed Paths Count        : %d / %d\n\n', total_failed_paths, num_paths);

% Save results to MAT file for evidence
save(fullfile(projectRoot, 'tests', 'adversarial_m3_continuity_results.mat'), 'results');

%% PART 2: Empirical Flaw and Bug Verification
fprintf('--------------------------------------------------------------------------------\n');
fprintf(' PART 2: Verification of Worker Test Flaws & Implementation Defects            \n');
fprintf('--------------------------------------------------------------------------------\n');

% Flaw 1: Worker's Test 10 self-subtraction diagnosis
wp_sample = [240, 0, 180, 0, 2.0; 220, -100, 120, 0, 2.0; 220, -100, 40, 0, 1.5];
traj_flaw = multi_point_planner(wp_sample, [], 0.01, 'cartesian');
tb_sample = cumsum(traj_flaw.segment_durations);
ib = find(traj_flaw.t <= tb_sample(1), 1, 'last');
ia = find(traj_flaw.t >= tb_sample(1), 1, 'first');
fprintf('[DEFECT A] Worker Test 10 Index Subtraction Flaw:\n');
fprintf('           At boundary tb=%.2fs: idx_before=%d (t=%.2f), idx_after=%d (t=%.2f)\n', ...
    tb_sample(1), ib, traj_flaw.t(ib), ia, traj_flaw.t(ia));
assert(ib == ia, 'Expected worker test 10 indices to be identical due to bug.');
fprintf('           CONFIRMED: Worker Test 10 subtracted identical index %d from %d (self-subtraction).\n\n', ib, ia);

% Bug 2: multi_point_planner.m crash on unreachable Cartesian waypoint
fprintf('[DEFECT B] multi_point_planner.m Line 131 Empty Assignment Bug:\n');
wp_unreachable = [500, 500, 500, 0; 200, 0, 100, 0];
crashed = false;
err_msg = '';
try
    multi_point_planner(wp_unreachable);
catch ME
    crashed = true;
    err_msg = ME.message;
end
fprintf('           Feeding unreachable Cartesian waypoint [500, 500, 500, 0] ...\n');
fprintf('           Crashed: %s\n', mat2str(crashed));
fprintf('           Verbatim error: "%s"\n', err_msg);
assert(crashed && contains(err_msg, '0-by-0'), 'Expected line 131 0-by-0 assignment error.');
fprintf('           CONFIRMED: multi_point_planner crashes on unreachable points instead of returning is_valid=false.\n\n');

if total_failed_paths == 0
    fprintf('>>> VERDICT ON CONTINUITY CRITERIA: ALL 50 PATHS PASS CONTINUITY (vel_jump < 1e-3, acc_jump < 1e-3) <<<\n');
else
    error('test_challenger_m3_1:ContinuityTestFailed', '%d paths failed continuity or velocity limits.', total_failed_paths);
end
