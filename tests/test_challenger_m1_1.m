% TEST_CHALLENGER_M1_1 Empirical Adversarial Challenge Test for Simscape Multibody
% Evaluates mypalletizer_simscape.slx against pure MATLAB forward_kinematics.m
% Cross-verifies Transform Sensor readings vs Analytical DH Kinematics
% Stress-tests high-speed dynamic tracking

fprintf('================================================================================\n');
fprintf('  ADVERSARIAL VERIFICATION SUITE: Simscape vs Analytical Forward Kinematics     \n');
fprintf('  Challenger: teamwork_challenger (Challenger M1-1)                             \n');
fprintf('================================================================================\n\n');

projectRoot = 'd:\Lidar\myPalletizer_BTL';
addpath(fullfile(projectRoot, 'models'));
addpath(fullfile(projectRoot, 'models', 'urdf'));
addpath(fullfile(projectRoot, 'models', 'meshes'));
addpath(fullfile(projectRoot, 'kinematics'));
addpath(fullfile(projectRoot, 'trajectory'));
addpath(fullfile(projectRoot, 'tests'));

modelName = 'mypalletizer_simscape';
modelPath = fullfile(projectRoot, 'models', [modelName, '.slx']);

if ~exist(modelPath, 'file')
    error('test_challenger_m1_1:ModelNotFound', 'Model file not found: %s', modelPath);
end

% Ensure model is loaded
if ~bdIsLoaded(modelName)
    load_system(modelPath);
end

%% PART 1: Multi-Configuration Static & Steady-State Cross-Verification
fprintf('--------------------------------------------------------------------------------\n');
fprintf(' PART 1: Multi-Configuration Simscape vs Analytical FK Cross-Verification       \n');
fprintf('--------------------------------------------------------------------------------\n');

% Test configurations spanning the admissible workspace (in radians)
% Joint limits:
%   q1: [-2.824, +2.824] rad
%   q2: [-0.0349, +1.5708] rad
%   q3: [-1.6057, +1.0471] rad
%   q4: [-3.14, +3.14] rad

test_configs = [
    0.0,                    0.0,                    0.0,                    0.0;                    % 1: Zero / Home pose
    deg2rad(45.0),          deg2rad(30.0),          deg2rad(-45.0),         deg2rad(15.0);          % 2: Pick stance
    deg2rad(-60.0),         deg2rad(60.0),          deg2rad(-60.0),         deg2rad(-30.0);         % 3: High reach
    deg2rad(90.0),          deg2rad(15.0),          deg2rad(-30.0),         deg2rad(45.0);          % 4: Right quadrant reach
    deg2rad(-90.0),         deg2rad(45.0),          deg2rad(-75.0),         deg2rad(-45.0);         % 5: Left quadrant reach
    0.0,                    deg2rad(75.0),          deg2rad(-90.0),         0.0;                    % 6: Deep fold / compact
    deg2rad(120.0),         deg2rad(10.0),          deg2rad(-20.0),         deg2rad(90.0);          % 7: Extreme yaw positive
    deg2rad(-120.0),        deg2rad(20.0),          deg2rad(-40.0),         deg2rad(-90.0);         % 8: Extreme yaw negative
    deg2rad(0.0),           deg2rad(45.0),          deg2rad(0.0),           deg2rad(0.0);           % 9: Upright elbow
    deg2rad(30.0),          deg2rad(0.0),           deg2rad(-60.0),         deg2rad(60.0);          % 10: Low shoulder
];

config_labels = {
    'Canonical Zero / Home Pose', ...
    'Pick Stance (mid-range)', ...
    'High Reach (steep shoulder)', ...
    'Right Quadrant Reach (+90 deg yaw)', ...
    'Left Quadrant Reach (-90 deg yaw)', ...
    'Deep Fold / Compact Stance', ...
    'Extreme Yaw Positive (+120 deg)', ...
    'Extreme Yaw Negative (-120 deg)', ...
    'Upright Shoulder (45 deg, elbow 0)', ...
    'Low Shoulder (0 deg, elbow -60)' ...
};

num_configs = size(test_configs, 1);
results_part1 = repmat(struct('Config', [], 'Label', '', 'Sim_m', [], 'FK_mm', [], ...
                              'Sim_mm', [], 'Discrepancy_mm', 0, 'Pass', false), num_configs, 1);

fprintf('%-4s | %-32s | %-20s | %-20s | %-12s | %-6s\n', ...
    'No.', 'Configuration Description', 'Simscape EE [mm]', 'Analytical FK [mm]', 'Error (mm)', 'Verdict');
fprintf('----------------------------------------------------------------------------------------------------------------\n');

set_param(modelName, 'StopTime', '0.2');

for i = 1:num_configs
    q_i = test_configs(i, :);
    label_i = config_labels{i};
    
    % Analytical FK in mm
    pose_fk = forward_kinematics(q_i);
    pos_fk_mm = pose_fk(1:3);
    
    % Build dataset with constant value held for 0.2s
    t_span = [0; 0.2];
    q_span = [q_i; q_i];
    
    ds = Simulink.SimulationData.Dataset;
    ds{1} = timeseries(q_span(:, 1), t_span, 'Name', 'q1');
    ds{2} = timeseries(q_span(:, 2), t_span, 'Name', 'q2');
    ds{3} = timeseries(q_span(:, 3), t_span, 'Name', 'q3');
    ds{4} = timeseries(q_span(:, 4), t_span, 'Name', 'q4');
    
    assignin('base', 'sim_adversarial_in', ds);
    
    % Run Simscape simulation
    try
        simOut = sim(modelName, 'LoadExternalInput', 'on', 'ExternalInput', 'sim_adversarial_in');
        ee_data = simOut.yout{1}.Values.Data;
        if ndims(ee_data) == 3
            ee_data = squeeze(ee_data)';
        elseif size(ee_data, 1) == 3 && size(ee_data, 2) ~= 3
            ee_data = ee_data';
        end
        
        % Final steady-state position in meters, converted to mm
        pos_sim_mm = ee_data(end, :) * 1000.0;
        
        disc_mm = norm(pos_sim_mm - pos_fk_mm);
        passed = (disc_mm < 1.0);
        
        sim_str = sprintf('[%6.1f, %6.1f, %6.1f]', pos_sim_mm);
        fk_str  = sprintf('[%6.1f, %6.1f, %6.1f]', pos_fk_mm);
        if passed
            verdict_str = 'PASS';
        else
            verdict_str = 'FAIL';
        end
        
        fprintf('%-4d | %-32s | %-20s | %-20s | %10.2f mm | %-6s\n', ...
            i, label_i, sim_str, fk_str, disc_mm, verdict_str);
        
        results_part1(i).Config = q_i;
        results_part1(i).Label = label_i;
        results_part1(i).Sim_m = ee_data(end, :);
        results_part1(i).Sim_mm = pos_sim_mm;
        results_part1(i).FK_mm = pos_fk_mm;
        results_part1(i).Discrepancy_mm = disc_mm;
        results_part1(i).Pass = passed;
        
    catch ME
        fprintf('%-4d | %-32s | SIMULATION CRASH: %s\n', i, label_i, ME.message);
        results_part1(i).Config = q_i;
        results_part1(i).Label = label_i;
        results_part1(i).Pass = false;
        results_part1(i).Discrepancy_mm = NaN;
    end
end
fprintf('----------------------------------------------------------------------------------------------------------------\n\n');

%% PART 2: High-Speed Dynamic Tracking & Solver Stress-Testing
fprintf('--------------------------------------------------------------------------------\n');
fprintf(' PART 2: High-Speed Dynamic Tracking & Solver Stress-Testing                    \n');
fprintf('--------------------------------------------------------------------------------\n');

dynamic_cases = {
    struct('name', 'Nominal Speed (omega = 1.0 rad/s)', ...
           'omega', 1.0, 'T', 2.0, ...
           'A1', 0.5, 'A2', 0.3, 'A3', 0.4, 'A4', 0.5), ...
    struct('name', 'Dispatch High Speed (omega = 2.0 rad/s)', ...
           'omega', 2.0, 'T', 2.0, ...
           'A1', 0.8, 'A2', 0.4, 'A3', 0.5, 'A4', 0.8), ...
    struct('name', 'Near-Limit Extreme Speed (omega = 3.5 rad/s, V_joint ~ 1.8 rad/s)', ...
           'omega', 3.5, 'T', 2.0, ...
           'A1', 0.5, 'A2', 0.3, 'A3', 0.3, 'A4', 0.5), ...
    struct('name', 'Rapid Multi-Frequency Chirp / Reversal', ...
           'omega', 5.0, 'T', 2.0, ...
           'A1', 0.3, 'A2', 0.2, 'A3', 0.2, 'A4', 0.3)
};

num_dyn = length(dynamic_cases);
results_part2 = repmat(struct('Name', '', 'SolverCompleted', false, 'MaxTrackingErr_rad', 0, ...
                              'MaxEESpeed_mps', 0, 'HasNaNOrInf', true, 'Pass', false), num_dyn, 1);

for k = 1:num_dyn
    dcase = dynamic_cases{k};
    fprintf('Executing Scenario %d: %s\n', k, dcase.name);
    
    t_end = dcase.T;
    dt = 0.002; % 500 Hz commanded trajectory
    t_vec = (0:dt:t_end)';
    w = dcase.omega;
    
    % Smooth sinusoidal trajectory respecting physical joint limits:
    % q1: [-2.824, 2.824] -> offset 0.0, amplitude A1
    q1_cmd = dcase.A1 * sin(w * t_vec);
    % q2: [-0.0349, 1.5708] -> offset 0.4, amplitude A2
    q2_cmd = 0.4 + dcase.A2 * sin(w * t_vec);
    % q3: [-1.6057, 1.0471] -> offset -0.4, amplitude A3
    q3_cmd = -0.4 + dcase.A3 * cos(w * t_vec);
    % q4: [-3.14, 3.14] -> offset 0.0, amplitude A4
    q4_cmd = dcase.A4 * sin(w * t_vec + pi/4);
    
    cmd_matrix = [q1_cmd, q2_cmd, q3_cmd, q4_cmd];
    
    % Max commanded joint velocities
    dq1_max = max(abs(diff(q1_cmd)/dt));
    dq2_max = max(abs(diff(q2_cmd)/dt));
    dq3_max = max(abs(diff(q3_cmd)/dt));
    dq4_max = max(abs(diff(q4_cmd)/dt));
    max_cmd_vel = max([dq1_max, dq2_max, dq3_max, dq4_max]);
    
    fprintf('  Commanded Peak Joint Velocity: %.3f rad/s (%.1f deg/s)\n', ...
        max_cmd_vel, rad2deg(max_cmd_vel));
    
    % Construct Dataset
    ds_dyn = Simulink.SimulationData.Dataset;
    ds_dyn{1} = timeseries(q1_cmd, t_vec, 'Name', 'q1');
    ds_dyn{2} = timeseries(q2_cmd, t_vec, 'Name', 'q2');
    ds_dyn{3} = timeseries(q3_cmd, t_vec, 'Name', 'q3');
    ds_dyn{4} = timeseries(q4_cmd, t_vec, 'Name', 'q4');
    
    assignin('base', 'sim_dyn_in', ds_dyn);
    set_param(modelName, 'StopTime', num2str(t_end));
    
    t_sim_start = tic;
    try
        simOut_dyn = sim(modelName, 'LoadExternalInput', 'on', 'ExternalInput', 'sim_dyn_in');
        t_elapsed = toc(t_sim_start);
        
        ee_dyn = simOut_dyn.yout{1}.Values.Data;
        if ndims(ee_dyn) == 3, ee_dyn = squeeze(ee_dyn)'; end
        if size(ee_dyn, 1) == 3 && size(ee_dyn, 2) ~= 3, ee_dyn = ee_dyn'; end
        
        q_meas = simOut_dyn.yout{2}.Values.Data;
        if ndims(q_meas) == 3, q_meas = squeeze(q_meas)'; end
        if size(q_meas, 1) == 4 && size(q_meas, 2) ~= 4, q_meas = q_meas'; end
        
        t_solver = simOut_dyn.tout;
        
        % Check for NaN or Inf
        has_nan_inf = any(isnan(ee_dyn(:))) || any(isinf(ee_dyn(:))) || ...
                      any(isnan(q_meas(:))) || any(isinf(q_meas(:)));
                  
        % Evaluate tracking error after 0.05s initial filter transient
        eval_idx = find(t_solver >= 0.05);
        q_interp_cmd = interp1(t_vec, cmd_matrix, t_solver(eval_idx));
        q_meas_eval = q_meas(eval_idx, :);
        
        joint_tracking_err = max(abs(q_meas_eval - q_interp_cmd));
        max_track_err = max(joint_tracking_err);
        
        % Compute EE speed
        dt_actual = diff(t_solver);
        delta_ee = diff(ee_dyn);
        ee_speed = sqrt(sum(delta_ee.^2, 2)) ./ dt_actual;
        max_ee_speed = max(ee_speed);
        
        solver_ok = (simOut_dyn.tout(end) >= t_end - 1e-4);
        test_passed = solver_ok && ~has_nan_inf && (max_track_err < 0.05);
        
        fprintf('  Simulation Time: %.3f s (wall clock: %.2f s, %d steps)\n', ...
            t_end, t_elapsed, length(t_solver));
        fprintf('  Max Joint Tracking Error: %.4f rad (%.3f deg)\n', ...
            max_track_err, rad2deg(max_track_err));
        fprintf('  Max End-Effector Speed  : %.3f m/s\n', max_ee_speed);
        fprintf('  Numerical Divergence/NaN: %s\n', mat2str(has_nan_inf));
        if test_passed
            fprintf('  -> VERDICT: PASS\n\n');
        else
            fprintf('  -> VERDICT: FAIL\n\n');
        end
        
        results_part2(k).Name = dcase.name;
        results_part2(k).SolverCompleted = solver_ok;
        results_part2(k).MaxTrackingErr_rad = max_track_err;
        results_part2(k).MaxEESpeed_mps = max_ee_speed;
        results_part2(k).HasNaNOrInf = has_nan_inf;
        results_part2(k).Pass = test_passed;
        
    catch ME
        fprintf('  -> SIMULATION CRASH / SOLVER FAILURE: %s\n\n', ME.message);
        results_part2(k).Name = dcase.name;
        results_part2(k).SolverCompleted = false;
        results_part2(k).Pass = false;
    end
end

%% Clean up
close_system(modelName, 0);
evalin('base', 'clear sim_adversarial_in sim_dyn_in');

% Save results MAT file for archiving
results_file = fullfile(projectRoot, 'tests', 'adversarial_m1_1_results.mat');
save(results_file, 'results_part1', 'results_part2');
fprintf('Empirical results saved to: %s\n', results_file);
