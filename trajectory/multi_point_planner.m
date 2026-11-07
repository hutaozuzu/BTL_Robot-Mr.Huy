function traj_struct = multi_point_planner(waypoints, durations, dt, waypoint_type, options)
% MULTI_POINT_PLANNER Multi-segment trajectory planner with C^2 continuity and velocity limits.
%
% Plans a smooth, continuous trajectory across N via-points for the 4-DOF
% myPalletizer 260 Pi robot. Supports Cartesian [x, y, z, phi] via-points
% or direct joint waypoints, enforces velocity limits (<= 120 deg/s), and
% ensures zero acceleration jump across segment boundaries.
%
% Syntax:
%   traj_struct = multi_point_planner(waypoints)
%   traj_struct = multi_point_planner(waypoints, durations)
%   traj_struct = multi_point_planner(waypoints, durations, dt)
%   traj_struct = multi_point_planner(waypoints, durations, dt, waypoint_type)
%   traj_struct = multi_point_planner(waypoints, durations, dt, waypoint_type, options)
%
% Inputs:
%   waypoints     - [Nx4] or [Nx5] matrix of via-points:
%                   If 4 columns: [x, y, z, phi] in mm and rad, or [th1, th2, th3, th4] in rad.
%                   If 5 columns: 5th column contains per-segment/waypoint durations (s).
%   durations     - Optional [(N-1)x1] or scalar segment durations (seconds).
%   dt            - Optional time step (seconds, default: 0.02 s).
%   waypoint_type - Optional: 'auto' (default), 'cartesian', or 'joint'.
%   options       - Optional struct with fields:
%                     .auto_adjust_duration - auto-stretch duration to respect vel limits (default: true)
%                     .max_joint_vel        - max joint speed (default: 120 deg/s = 2.0944 rad/s)
%                     .safety_factor        - scaling headroom factor (default: 1.05)
%                     .config_pref          - 'closest' (default), 'elbow_up', 'elbow_down'
%
% Outputs:
%   traj_struct   - Struct containing trajectory data:
%                     .t                 - [Mx1] strictly monotonic time vector
%                     .theta             - [Mx4] joint positions (rad)
%                     .dtheta            - [Mx4] joint velocities (rad/s)
%                     .ddtheta           - [Mx4] joint accelerations (rad/s^2)
%                     .pose              - [Mx4] Cartesian pose [x, y, z, phi] (mm, rad)
%                     .xyz_actual        - [Mx3] Cartesian position [x, y, z] (mm)
%                     .is_valid          - Logical flag (true if valid and in-bounds)
%                     .segment_durations - [(N-1)x1] durations per segment (s)
%                     .total_duration    - Total trajectory execution time (s)
%                     .max_velocity      - Peak observed joint velocity (rad/s)
%                     .max_accel         - Peak observed joint acceleration (rad/s^2)
%                     .waypoints_joint   - [Nx4] resolved joint waypoints
%                     .dt                - Sampling step
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    if nargin < 1 || isempty(waypoints)
        error('multi_point_planner:EmptyWaypoints', 'Waypoints matrix cannot be empty.');
    end

    % Parse options
    if nargin < 5 || isempty(options), options = struct(); end
    if ~isfield(options, 'auto_adjust_duration'), options.auto_adjust_duration = true; end
    if ~isfield(options, 'max_joint_vel'), options.max_joint_vel = deg2rad(120.0); end % ~2.0944 rad/s
    if ~isfield(options, 'safety_factor'), options.safety_factor = 1.05; end
    if ~isfield(options, 'config_pref'), options.config_pref = 'closest'; end

    % Sampling interval
    if nargin < 3 || isempty(dt)
        dt = 0.02; % 50 Hz default
    end
    if dt <= 0
        error('multi_point_planner:InvalidDt', 'Sampling step dt must be strictly positive.');
    end

    num_wp = size(waypoints, 1);
    num_cols = size(waypoints, 2);

    if num_wp < 2
        error('multi_point_planner:TooFewWaypoints', 'At least 2 waypoints are required.');
    end

    % Extract durations if embedded in 5th column
    if num_cols >= 5 && (nargin < 2 || isempty(durations))
        embedded_dur = waypoints(:, 5);
        durations = embedded_dur(1:(num_wp - 1));
    end

    % Default segment durations if not provided
    if nargin < 2 || isempty(durations)
        durations = 2.0 * ones(num_wp - 1, 1);
    elseif isscalar(durations)
        durations = repmat(durations, num_wp - 1, 1);
    else
        durations = durations(:);
        if numel(durations) == num_wp
            durations = durations(1:(num_wp - 1));
        elseif numel(durations) ~= (num_wp - 1)
            error('multi_point_planner:InvalidDurations', ...
                  'Durations must have N-1 elements for N waypoints.');
        end
    end

    if any(durations <= 0)
        error('multi_point_planner:InvalidDurations', 'All durations must be strictly positive.');
    end

    % Determine waypoint type
    pts = waypoints(:, 1:4);
    if nargin < 4 || isempty(waypoint_type) || strcmpi(waypoint_type, 'auto')
        % If coordinates have values > 2*pi, they must be Cartesian mm
        mean_radius = mean(sqrt(pts(:, 1).^2 + pts(:, 2).^2));
        if mean_radius > 10.0 || max(abs(pts(:))) > 10.0
            waypoint_type = 'cartesian';
        else
            waypoint_type = 'joint';
        end
    else
        waypoint_type = lower(waypoint_type);
    end

    % Convert to joint space if Cartesian
    q_wp = zeros(num_wp, 4);
    is_valid = true;
    error_msg = '';

    if strcmpi(waypoint_type, 'cartesian')
        q_prev = [0.0, 0.0, 0.0, 0.0];
        for i = 1:num_wp
            px = pts(i, 1);
            py = pts(i, 2);
            pz = pts(i, 3);
            pphi = pts(i, 4);
            [th_opt, success] = inverse_kinematics(px, py, pz, pphi, options.config_pref, q_prev);
            if ~success
                is_valid = false;
                error_msg = sprintf('Cartesian waypoint %d [%.1f, %.1f, %.1f, %.1f] is unreachable.', ...
                                    i, px, py, pz, pphi);
                th_opt = q_prev; % Assign fallback configuration to maintain valid matrix dimensions
            end
            q_wp(i, :) = th_opt;
            q_prev = th_opt;
        end
    else
        q_wp = pts;
        % Validate joint limits
        for i = 1:num_wp
            if ~check_limits(q_wp(i, :), 'joints')
                is_valid = false;
                error_msg = sprintf('Joint waypoint %d exceeds joint limits.', i);
            end
        end
    end

    % Velocity Limit Enforcement & Duration Scaling
    % For quintic polynomial with zero boundary velocity: v_peak = 1.875 * |delta_q| / T
    adj_durations = durations;
    for seg = 1:(num_wp - 1)
        dq = abs(q_wp(seg + 1, :) - q_wp(seg, :));
        v_peak = 1.875 * dq / adj_durations(seg);
        max_seg_vel = max(v_peak);
        if max_seg_vel > options.max_joint_vel
            if options.auto_adjust_duration
                min_T = max(1.875 * dq / options.max_joint_vel) * options.safety_factor;
                adj_durations(seg) = max(adj_durations(seg), min_T);
            else
                is_valid = false;
            end
        end
    end

    % Interpolate segments using quintic polynomial
    t_cells     = cell(num_wp - 1, 1);
    theta_cells = cell(num_wp - 1, 1);
    dth_cells   = cell(num_wp - 1, 1);
    ddth_cells  = cell(num_wp - 1, 1);

    t_accum = 0.0;
    for seg = 1:(num_wp - 1)
        T_seg = adj_durations(seg);
        t_start = t_accum;
        t_end   = t_accum + T_seg;

        if seg == 1
            t_seg = (t_start : dt : t_end)';
            if isempty(t_seg) || t_seg(end) < t_end - 1e-9
                t_seg = [t_seg; t_end];
            end
        else
            % Exclude boundary start point to prevent duplicate timestamps
            t_seg = (t_start + dt : dt : t_end)';
            if isempty(t_seg) || t_seg(end) < t_end - 1e-9
                t_seg = [t_seg; t_end];
            end
        end

        % Compute 4-joint quintic trajectories
        [q_s, q_ds, q_dds] = quintic_trajectory(q_wp(seg, :), q_wp(seg + 1, :), ...
                                                t_start, t_end, t_seg);

        t_cells{seg}     = t_seg;
        theta_cells{seg} = q_s;
        dth_cells{seg}   = q_ds;
        ddth_cells{seg}  = q_dds;

        t_accum = t_end;
    end

    % Concatenate into continuous trajectories
    t_full     = cell2mat(t_cells);
    theta_full = cell2mat(theta_cells);
    dth_full   = cell2mat(dth_cells);
    ddth_full  = cell2mat(ddth_cells);
    M = numel(t_full);

    % Compute Cartesian forward kinematics along full trajectory
    pose_full = zeros(M, 4);
    for m = 1:M
        p_m = forward_kinematics(theta_full(m, :));
        pose_full(m, :) = p_m;
        % Check joint bounds
        if is_valid && ~check_limits(theta_full(m, :), 'joints')
            is_valid = false;
            error_msg = sprintf('Trajectory point at t=%.3fs violates joint limits.', t_full(m));
        end
    end

    % Pack output struct
    traj_struct = struct();
    traj_struct.t                 = t_full;
    traj_struct.theta             = theta_full;
    traj_struct.dtheta            = dth_full;
    traj_struct.ddtheta           = ddth_full;
    traj_struct.pose              = pose_full;
    traj_struct.xyz_actual        = pose_full(:, 1:3);
    traj_struct.is_valid          = is_valid;
    traj_struct.segment_durations = adj_durations;
    traj_struct.total_duration    = t_accum;
    traj_struct.max_velocity      = max(abs(dth_full(:)));
    traj_struct.max_accel         = max(abs(ddth_full(:)));
    traj_struct.waypoints_joint   = q_wp;
    traj_struct.dt                = dt;
    if ~isempty(error_msg)
        traj_struct.error_msg     = error_msg;
    end
end
