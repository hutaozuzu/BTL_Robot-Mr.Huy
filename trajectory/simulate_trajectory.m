function [sim_results, stream_data] = simulate_trajectory(traj_input, sim_options)
% SIMULATE_TRAJECTORY Feeds planned trajectory to simulation and logs tracking performance.
%
% Prepares Simulink/Simscape From-Workspace time-series structures, executes
% physical or kinematic simulation, and computes tracking error metrics
% between desired commands and actual manipulator motion.
%
% Syntax:
%   [sim_results, stream_data] = simulate_trajectory(traj_input)
%   [sim_results, stream_data] = simulate_trajectory(traj_input, sim_options)
%
% Inputs:
%   traj_input  - Trajectory struct from multi_point_planner, OR [Mx4] matrix
%                 of joint angles with optional time vector
%   sim_options - Optional struct with fields:
%                   .model_name    - Simulink model name (default: 'mypalletizer_simscape')
%                   .run_simscape  - Attempt Simulink model execution (default: false)
%                   .noise_std     - Simulated sensor noise std in radians (default: 0)
%                   .dt            - Time step if traj_input is a matrix (default: 0.02s)
%
% Outputs:
%   sim_results - Struct containing execution logs and analytics:
%                   .time                    - [Mx1] simulation time vector
%                   .q_desired               - [Mx4] commanded joint angles (rad)
%                   .q_actual                - [Mx4] simulated/sensed joint angles (rad)
%                   .dq_desired              - [Mx4] commanded joint velocities (rad/s)
%                   .dq_actual               - [Mx4] simulated/sensed joint velocities (rad/s)
%                   .cartesian_desired       - [Mx4] commanded [x, y, z, phi] (mm, rad)
%                   .cartesian_actual        - [Mx4] actual [x, y, z, phi] (mm, rad)
%                   .tracking_error_joint    - [Mx4] joint angle deviations (rad)
%                   .tracking_error_cartesian- [Mx1] 3D Cartesian Euclidean error (mm)
%                   .max_cartesian_error     - Peak Cartesian tracking error (mm)
%                   .mean_cartesian_error    - Mean Cartesian tracking error (mm)
%                   .max_joint_velocity      - Peak observed joint velocity (rad/s)
%                   .is_velocity_compliant   - Logical flag (<= 120 deg/s)
%                   .is_tracking_compliant   - Logical flag (error < 1.0 mm)
%   stream_data - Struct formatted for Simulink From-Workspace block:
%                   .time                    - [Mx1] time vector
%                   .signals.values          - [Mx4] joint trajectory values
%                   .signals.dimensions      - 4
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    if nargin < 1 || isempty(traj_input)
        error('simulate_trajectory:EmptyInput', 'Trajectory input cannot be empty.');
    end

    if nargin < 2 || isempty(sim_options), sim_options = struct(); end
    if ~isfield(sim_options, 'model_name'), sim_options.model_name = 'mypalletizer_simscape'; end
    if ~isfield(sim_options, 'run_simscape'), sim_options.run_simscape = false; end
    if ~isfield(sim_options, 'noise_std'), sim_options.noise_std = 0.0; end
    if ~isfield(sim_options, 'dt'), sim_options.dt = 0.02; end

    % Extract time, theta, dtheta from input
    if isstruct(traj_input)
        if isfield(traj_input, 't')
            t_vec = traj_input.t(:);
        elseif isfield(traj_input, 'time')
            t_vec = traj_input.time(:);
        else
            M = size(traj_input.theta, 1);
            t_vec = (0 : (M - 1))' * sim_options.dt;
        end

        if isfield(traj_input, 'theta')
            q_des = traj_input.theta;
        elseif isfield(traj_input, 'signals') && isfield(traj_input.signals, 'values')
            q_des = traj_input.signals.values;
        else
            error('simulate_trajectory:InvalidStruct', 'Struct must contain theta or signals.values.');
        end

        if isfield(traj_input, 'dtheta')
            dq_des = traj_input.dtheta;
        else
            dt = mean(diff(t_vec));
            if dt > 0
                dq_des = [zeros(1, 4); diff(q_des) / dt];
            else
                dq_des = zeros(size(q_des));
            end
        end

        if isfield(traj_input, 'pose')
            cart_des = traj_input.pose;
        else
            cart_des = zeros(size(q_des, 1), 4);
            for i = 1:size(q_des, 1)
                cart_des(i, :) = forward_kinematics(q_des(i, :));
            end
        end
    else
        % Numerical matrix input
        q_des = traj_input;
        M = size(q_des, 1);
        t_vec = (0 : (M - 1))' * sim_options.dt;
        dt = sim_options.dt;
        if dt > 0
            dq_des = [zeros(1, 4); diff(q_des) / dt];
        else
            dq_des = zeros(size(q_des));
        end
        cart_des = zeros(M, 4);
        for i = 1:M
            cart_des(i, :) = forward_kinematics(q_des(i, :));
        end
    end

    M = numel(t_vec);

    % Prepare Simulink From-Workspace data structure
    stream_data = struct();
    stream_data.time = t_vec;
    stream_data.signals = struct();
    stream_data.signals.values = q_des;
    stream_data.signals.dimensions = size(q_des, 2);

    % If Simulink SimulationData.Dataset is available, create 4-channel dataset for Inports q1..q4
    try
        ds = Simulink.SimulationData.Dataset();
        for ch = 1:4
            ds{ch} = timeseries(q_des(:, ch), t_vec, 'Name', sprintf('q%d', ch));
        end
        stream_data.dataset = ds;
    catch
        % Dataset creation optional in non-Simulink environments
    end

    % Execute simulation or kinematic tracking
    q_act = q_des;
    dq_act = dq_des;

    simscape_executed = false;
    if sim_options.run_simscape
        % Check if Simscape model is on path and loadable
        model_file = which(sim_options.model_name);
        if ~isempty(model_file)
            try
                if ~bdIsLoaded(sim_options.model_name)
                    load_system(model_file);
                end
                % Assign input dataset to base workspace
                assignin('base', 'theta_trajectory', stream_data);
                if isfield(stream_data, 'dataset')
                    assignin('base', 'sim_traj_dataset', stream_data.dataset);
                    sim_out = sim(sim_options.model_name, ...
                        'StopTime', num2str(t_vec(end)), ...
                        'LoadExternalInput', 'on', ...
                        'ExternalInput', 'sim_traj_dataset');
                else
                    sim_out = sim(sim_options.model_name, 'StopTime', num2str(t_vec(end)));
                end
                if isprop(sim_out, 'tout') && ~isempty(sim_out.tout)
                    simscape_executed = true;
                    % Extract logged signals if available from Outport 2 (q_meas)
                    if isprop(sim_out, 'yout') && ~isempty(sim_out.yout) && numElements(sim_out.yout) >= 2
                        q_sensed_raw = sim_out.yout{2}.Values.Data;
                        if ndims(q_sensed_raw) == 3
                            q_sensed_raw = squeeze(q_sensed_raw)';
                        elseif size(q_sensed_raw, 1) == 4 && size(q_sensed_raw, 2) ~= 4
                            q_sensed_raw = q_sensed_raw';
                        end
                        q_act = interp1(sim_out.tout, q_sensed_raw, t_vec, 'linear', 'extrap');
                    elseif isprop(sim_out, 'q_sensed')
                        q_act = interp1(sim_out.tout, sim_out.q_sensed, t_vec, 'linear', 'extrap');
                    end
                end
            catch
                simscape_executed = false;
            end
        end
    end

    % If not run through Simscape or noise injection requested
    if ~simscape_executed && sim_options.noise_std > 0
        noise = sim_options.noise_std * randn(size(q_des));
        q_act = q_des + noise;
        dq_act = dq_des + sim_options.noise_std * randn(size(dq_des));
    end

    % Calculate actual Cartesian pose via forward kinematics
    cart_act = zeros(M, 4);
    for i = 1:M
        cart_act(i, :) = forward_kinematics(q_act(i, :));
    end

    % Tracking error metrics
    tracking_err_joint = q_act - q_des;
    % 3D Euclidean Cartesian position error ||P_des - P_act||
    pos_err_3d = sqrt(sum((cart_des(:, 1:3) - cart_act(:, 1:3)).^2, 2));

    max_cart_err  = max(pos_err_3d);
    mean_cart_err = mean(pos_err_3d);
    max_vel       = max(abs(dq_act(:)));

    MAX_VEL_RAD = deg2rad(120.0); % 2.0944 rad/s

    % Construct results struct
    sim_results = struct();
    sim_results.time                     = t_vec;
    sim_results.q_desired                = q_des;
    sim_results.q_actual                 = q_act;
    sim_results.dq_desired               = dq_des;
    sim_results.dq_actual                = dq_act;
    sim_results.cartesian_desired        = cart_des;
    sim_results.cartesian_actual         = cart_act;
    sim_results.tracking_error_joint     = tracking_err_joint;
    sim_results.tracking_error_cartesian = pos_err_3d;
    sim_results.max_cartesian_error      = max_cart_err;
    sim_results.mean_cartesian_error     = mean_cart_err;
    sim_results.max_joint_velocity       = max_vel;
    sim_results.is_velocity_compliant    = (max_vel <= MAX_VEL_RAD + 1e-4);
    sim_results.is_tracking_compliant    = (max_cart_err < 1.0);
    sim_results.simscape_executed        = simscape_executed;
end
