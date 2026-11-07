classdef test_utils
    % TEST_UTILS Helper utilities, oracles, and runners for myPalletizer 260 Pi E2E tests.
    % Strictly pure MATLAB - zero Peter Corke Robotics Toolbox dependencies.

    properties (Constant)
        % Robot geometric constants (mm)
        D1 = 120.0;     % Base height
        A2 = 140.0;     % Upper arm length
        A3 = 134.0;     % Forearm length
        A4 = 44.0;      % Wrist / tool flange length
        R_MAX = 260.0;  % Rated reach radius
        
        % Joint hardware limits (degrees and radians)
        JOINT_LIMITS_DEG = [ ...
            -162.0,  162.0;   % Joint 1: Base yaw
              -2.0,   90.0;   % Joint 2: Shoulder pitch
             -92.0,   60.0;   % Joint 3: Elbow pitch
            -180.0,  180.0    % Joint 4: Wrist pitch
        ];
        
        JOINT_LIMITS_RAD = [ ...
            -162.0 * pi/180,  162.0 * pi/180;
              -2.0 * pi/180,   90.0 * pi/180;
             -92.0 * pi/180,   60.0 * pi/180;
            -180.0 * pi/180,  180.0 * pi/180
        ];
        
        MAX_JOINT_VEL_DEG = 120.0;        % deg/s
        MAX_JOINT_VEL_RAD = 120.0 * pi/180; % rad/s
    end

    methods (Static)
        function base_dir = get_base_dir()
            % Return root directory of the myPalletizer_BTL project
            current_file = mfilename('fullpath');
            tests_dir = fileparts(current_file);
            base_dir = fileparts(tests_dir);
        end

        function add_project_paths()
            % Add all project subdirectories to MATLAB search path
            base_dir = test_utils.get_base_dir();
            subdirs = {'kinematics', 'trajectory', 'gui', 'models', 'data', 'tests'};
            for i = 1:numel(subdirs)
                d = fullfile(base_dir, subdirs{i});
                if exist(d, 'dir')
                    addpath(d);
                end
            end
        end

        function res = new_result(id, name, feature, tier, status, message, duration_ms)
            % Construct a standardized test result structure
            if nargin < 7, duration_ms = 0; end
            if nargin < 6, message = ''; end
            res.id = id;
            res.name = name;
            res.feature = feature;
            res.tier = tier;
            res.status = status; % 'PASS', 'FAIL', 'PENDING'
            res.message = message;
            res.duration_ms = duration_ms;
        end

        function [pose, T_all] = oracle_fk(theta)
            % Reference analytical Forward Kinematics
            % theta: [1x4] in radians
            theta = theta(:)';
            t1 = theta(1);
            t2 = theta(2);
            t3 = theta(3);
            t4 = theta(4);
            
            d1 = test_utils.D1;
            a2 = test_utils.A2;
            a3 = test_utils.A3;
            a4 = test_utils.A4;
            
            t23 = t2 + t3;
            t234 = t2 + t3 + t4;
            
            r = a2 * cos(t2) + a3 * cos(t23) + a4 * cos(t234);
            x = r * cos(t1);
            y = r * sin(t1);
            z = d1 + a2 * sin(t2) + a3 * sin(t23) + a4 * sin(t234);
            phi = t234;
            
            pose = [x, y, z, phi];
            
            if nargout > 1
                % Compute individual Standard D-H homogeneous transformation matrices
                T01 = [cos(t1), 0,  sin(t1), 0;
                       sin(t1), 0, -cos(t1), 0;
                       0,       1,  0,       d1;
                       0,       0,  0,       1];
                   
                T12 = [cos(t2), -sin(t2), 0, a2*cos(t2);
                       sin(t2),  cos(t2), 0, a2*sin(t2);
                       0,        0,       1, 0;
                       0,        0,       0, 1];
                   
                T23 = [cos(t3), -sin(t3), 0, a3*cos(t3);
                       sin(t3),  cos(t3), 0, a3*sin(t3);
                       0,        0,       1, 0;
                       0,        0,       0, 1];
                   
                T34 = [cos(t4), -sin(t4), 0, a4*cos(t4);
                       sin(t4),  cos(t4), 0, a4*sin(t4);
                       0,        0,       1, 0;
                       0,        0,       0, 1];
                   
                T02 = T01 * T12;
                T03 = T02 * T23;
                T04 = T03 * T34;
                T_all = {T01, T02, T03, T04};
            end
        end

        function [theta, success, all_solutions] = oracle_ik(x, y, z, phi, config_pref)
            % Reference analytical Inverse Kinematics
            if nargin < 5, config_pref = 'elbow_up'; end
            d1 = test_utils.D1;
            a2 = test_utils.A2;
            a3 = test_utils.A3;
            a4 = test_utils.A4;
            
            theta = zeros(1, 4);
            success = false;
            all_solutions = [];
            
            % Step 1: Base angle theta1
            t1 = atan2(y, x);
            r = sqrt(x^2 + y^2);
            
            % Step 2: Wrist center (Joint 4)
            rw = r - a4 * cos(phi);
            zw = z - d1 - a4 * sin(phi);
            
            % Step 3: Law of cosines for theta3
            D = (rw^2 + zw^2 - a2^2 - a3^2) / (2 * a2 * a3);
            if abs(D) > 1.0 + 1e-9
                return; % Out of reach
            end
            D = max(-1.0, min(1.0, D));
            
            % Solutions for elbow-up (t3 <= 0) and elbow-down (t3 >= 0)
            sin_t3_up = -sqrt(max(0, 1 - D^2));
            sin_t3_down = sqrt(max(0, 1 - D^2));
            
            t3_up = atan2(sin_t3_up, D);
            t3_down = atan2(sin_t3_down, D);
            
            % Step 4: Shoulder angle theta2
            t2_up = atan2(zw, rw) - atan2(a3 * sin(t3_up), a2 + a3 * cos(t3_up));
            t2_down = atan2(zw, rw) - atan2(a3 * sin(t3_down), a2 + a3 * cos(t3_down));
            
            % Step 5: Wrist angle theta4
            t4_up = phi - t2_up - t3_up;
            t4_down = phi - t2_down - t3_down;
            
            sol_up = [t1, t2_up, t3_up, t4_up];
            sol_down = [t1, t2_down, t3_down, t4_down];
            
            % Step 6: Validate against joint limits
            valid_up = test_utils.is_within_limits(sol_up);
            valid_down = test_utils.is_within_limits(sol_down);
            
            sols = [];
            if valid_up, sols = [sols; sol_up]; end
            if valid_down, sols = [sols; sol_down]; end
            all_solutions = sols;
            
            if isempty(sols)
                % Neither is strictly inside limits, but return best branch if close
                if strcmpi(config_pref, 'elbow_up')
                    theta = sol_up;
                else
                    theta = sol_down;
                end
                success = false;
                return;
            end
            
            if strcmpi(config_pref, 'elbow_up') && valid_up
                theta = sol_up;
                success = true;
            elseif strcmpi(config_pref, 'elbow_down') && valid_down
                theta = sol_down;
                success = true;
            else
                theta = sols(1, :);
                success = true;
            end
        end

        function tf = is_within_limits(theta)
            % Check if joint vector [1x4] is within hardware limits
            limits = test_utils.JOINT_LIMITS_RAD;
            tf = true;
            for i = 1:4
                if theta(i) < limits(i, 1) - 1e-6 || theta(i) > limits(i, 2) + 1e-6
                    tf = false;
                    return;
                end
            end
        end

        function [J, det_J, is_singular] = oracle_jacobian(theta)
            % Reference analytical 4x4 Jacobian
            theta = theta(:)';
            t1 = theta(1);
            t2 = theta(2);
            t3 = theta(3);
            t4 = theta(4);
            
            a2 = test_utils.A2;
            a3 = test_utils.A3;
            a4 = test_utils.A4;
            
            t23 = t2 + t3;
            t234 = t2 + t3 + t4;
            
            r = a2 * cos(t2) + a3 * cos(t23) + a4 * cos(t234);
            u2 = -a2 * sin(t2) - a3 * sin(t23) - a4 * sin(t234);
            u3 = -a3 * sin(t23) - a4 * sin(t234);
            u4 = -a4 * sin(t234);
            
            w2 = r;
            w3 = a3 * cos(t23) + a4 * cos(t234);
            w4 = a4 * cos(t234);
            
            J = [ -r * sin(t1),  u2 * cos(t1),  u3 * cos(t1),  u4 * cos(t1);
                   r * cos(t1),  u2 * sin(t1),  u3 * sin(t1),  u4 * sin(t1);
                   0,            w2,            w3,            w4;
                   0,            1,             1,             1 ];
               
            det_J = -r * a2 * a3 * sin(t3);
            is_singular = abs(det_J) < 1e-4;
        end

        function J_dls = oracle_dls_inverse(J, lambda)
            % Damped Least Squares inverse
            if nargin < 2 || isempty(lambda)
                lambda = 0.05;
            end
            m = size(J, 1);
            J_dls = J' / (J * J' + lambda^2 * eye(m));
        end

        function [s, ds, dds] = oracle_quintic(s0, sf, t0, tf, t)
            % Reference 5th-order polynomial trajectory
            T = tf - t0;
            if T <= 0
                error('Time duration T must be strictly positive');
            end
            tau = (t - t0) / T;
            tau = max(0, min(1, tau)); % saturate to [0, 1]
            
            delta = sf - s0;
            s = s0 + delta * (10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5);
            ds = (delta / T) * (30 * tau.^2 - 60 * tau.^3 + 30 * tau.^4);
            dds = (delta / (T^2)) * (60 * tau - 180 * tau.^2 + 120 * tau.^3);
        end

        function grid_points = oracle_pallet_grid(origin, rows, cols, layers, spacing)
            % Compute M x N x K pallet drop coordinates
            % origin: [x0, y0, z0]
            % spacing: [dx, dy, dz]
            x0 = origin(1); y0 = origin(2); z0 = origin(3);
            dx = spacing(1); dy = spacing(2); dz = spacing(3);
            
            total_points = rows * cols * layers;
            grid_points = zeros(total_points, 3);
            idx = 1;
            
            for k = 1:layers
                for i = 1:rows
                    for j = 1:cols
                        gx = x0 + (i - 1) * dx;
                        gy = y0 + (j - 1) * dy;
                        gz = z0 + (k - 1) * dz;
                        grid_points(idx, :) = [gx, gy, gz];
                        idx = idx + 1;
                    end
                end
            end
        end

        function has_corke = check_file_corke_violation(filepath)
            % Scan a file to detect any Peter Corke toolbox usage
            has_corke = false;
            if ~exist(filepath, 'file')
                return;
            end
            fid = fopen(filepath, 'r');
            if fid < 0, return; end
            
            corke_signatures = { ...
                'SerialLink', 'Link(', 'Revolute(', 'Prismatic(', ...
                'fkine(', 'ikine(', 'ikunc(', 'jacob0(', 'jacobn(', ...
                'tr2rpy(', 'rpy2tr(', 'transl(', 'trotx(', 'troty(', 'trotz(', ...
                'ctraj(', 'jtraj(', 'qplot(', 'teach(' ...
            };
            
            while ~feof(fid)
                line = strtrim(fgetl(fid));
                if ischar(line) && ~isempty(line) && ~startsWith(line, '%')
                    for s = 1:numel(corke_signatures)
                        if contains(line, corke_signatures{s})
                            has_corke = true;
                            fclose(fid);
                            return;
                        end
                    end
                end
            end
            fclose(fid);
        end
    end
end
