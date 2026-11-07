function [theta_opt, success, all_solutions] = inverse_kinematics(varargin)
% INVERSE_KINEMATICS Closed-form geometric inverse kinematics solver for 4-DOF myPalletizer 260 Pi.
%
% Syntax:
%   theta = inverse_kinematics(x, y, z, phi)
%   theta = inverse_kinematics(pose)
%   [theta, success, all_solutions] = inverse_kinematics(x, y, z, phi, config_pref, theta_current)
%   [theta, success, all_solutions] = inverse_kinematics(pose, config_pref, theta_current)
%
% Inputs:
%   x, y, z       - Tool Center Point Cartesian position in mm
%   phi           - End-effector tool pitch angle (radians): phi = theta2 + theta3 + theta4
%                   (Or passed together as pose = [x, y, z, phi])
%   config_pref   - Optional string: 'elbow_up' | 'elbow_down' | 'closest' (default: 'closest')
%   theta_current - Optional [1x4] current joint configuration for continuity optimization
%
% Outputs (PROJECT.md Interface Contract):
%   theta_opt     - [1x4] optimal joint angles [theta1, theta2, theta3, theta4] in radians
%   success       - Logical flag: true if a valid limit-satisfying solution exists, false otherwise
%   all_solutions - [Kx4] matrix of all admissible configurations within hardware limits
%
% Robot Dimensions:
%   d1 = 120.0 mm, a2 = 140.0 mm, a3 = 134.0 mm, a4 = 44.0 mm
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Parse inputs
    if nargin < 1
        error('inverse_kinematics:TooFewInputs', 'At least target pose or (x, y, z, phi) required.');
    end

    if numel(varargin{1}) == 4
        % Called as inverse_kinematics(pose, ...)
        pose = reshape(varargin{1}, 1, 4);
        x = pose(1);
        y = pose(2);
        z = pose(3);
        phi = pose(4);
        idx_opt = 2;
    else
        % Called as inverse_kinematics(x, y, z, phi, ...)
        if nargin < 4
            error('inverse_kinematics:TooFewInputs', 'Must provide x, y, z, and phi.');
        end
        x = varargin{1};
        y = varargin{2};
        z = varargin{3};
        phi = varargin{4};
        idx_opt = 5;
    end

    config_pref = 'closest';
    theta_current = [0.0, 0.0, 0.0, 0.0];

    if nargin >= idx_opt && ~isempty(varargin{idx_opt})
        config_pref = lower(varargin{idx_opt});
    end
    if nargin >= (idx_opt + 1) && ~isempty(varargin{idx_opt + 1})
        theta_current = reshape(varargin{idx_opt + 1}, 1, 4);
    end

    % Link parameters
    d1 = 120.0;
    a2 = 140.0;
    a3 = 134.0;
    a4 = 44.0;

    % Step 1: Base yaw angle theta1
    r = sqrt(x^2 + y^2);
    if r < 1e-6
        % Robot wrist is along the central vertical axis Z0 (singularity)
        th1 = theta_current(1);
    else
        th1 = atan2(y, x);
    end

    % Step 2: Wrist center position in the arm sagittal plane
    rw = r - a4 * cos(phi);
    zw = z - d1 - a4 * sin(phi);

    % Step 3: Elbow angle theta3 via Law of Cosines
    rho_w_sq = rw^2 + zw^2;
    D = (rho_w_sq - a2^2 - a3^2) / (2 * a2 * a3);

    % Geometric reachability check
    reach_tol = 1e-7;
    if D > (1.0 + reach_tol) || D < (-1.0 - reach_tol)
        % Target point is geometrically outside the reachable sphere
        success = false;
        theta_opt = [];
        all_solutions = [];
        return;
    end

    % Clamp D to [-1, 1] to guard against tiny floating-point rounding
    D = max(-1.0, min(1.0, D));
    sin_th3_mag = sqrt(max(0.0, 1.0 - D^2));

    % Two geometric branches for theta3:
    % Branch 1: Elbow-up (sin_th3 <= 0, standard palletizer configuration)
    th3_up   = atan2(-sin_th3_mag, D);
    % Branch 2: Elbow-down (sin_th3 >= 0)
    th3_down = atan2(+sin_th3_mag, D);

    branches_th3 = [th3_up, th3_down];
    branch_names = {'elbow_up', 'elbow_down'};

    valid_candidates = [];
    valid_names = {};

    for b = 1:2
        th3_cand = branches_th3(b);

        % Step 4: Shoulder pitch angle theta2
        beta = atan2(zw, rw);
        psi  = atan2(a3 * sin(th3_cand), a2 + a3 * cos(th3_cand));
        th2_cand = beta - psi;

        % Step 5: Wrist pitch angle theta4
        th4_cand = phi - th2_cand - th3_cand;
        % Normalize theta4 to [-pi, pi]
        th4_cand = atan2(sin(th4_cand), cos(th4_cand));

        cand_sol = [th1, th2_cand, th3_cand, th4_cand];

        % Step 6: Hardware limits validation
        if check_limits(cand_sol, 'joints')
            valid_candidates = [valid_candidates; cand_sol]; %#ok<AGROW>
            valid_names{end+1} = branch_names{b}; %#ok<AGROW>
        end
    end

    % Select optimal solution from admissible candidates
    if isempty(valid_candidates)
        success = false;
        theta_opt = [];
        all_solutions = [];
    else
        success = true;
        all_solutions = valid_candidates;

        if strcmpi(config_pref, 'elbow_up') && any(strcmp(valid_names, 'elbow_up'))
            idx = find(strcmp(valid_names, 'elbow_up'), 1);
            theta_opt = valid_candidates(idx, :);
        elseif strcmpi(config_pref, 'elbow_down') && any(strcmp(valid_names, 'elbow_down'))
            idx = find(strcmp(valid_names, 'elbow_down'), 1);
            theta_opt = valid_candidates(idx, :);
        else
            % Default: Closest configuration to theta_current (weighted joint norm)
            w = [1.0, 1.2, 1.0, 0.8];
            diff = valid_candidates - theta_current;
            cost = sum((diff.^2) .* w, 2);
            [~, min_idx] = min(cost);
            theta_opt = valid_candidates(min_idx, :);
        end
    end
end
