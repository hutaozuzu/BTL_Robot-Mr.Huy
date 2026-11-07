% TEST_JACOBIAN_GRADIENT Numerical gradient verification of analytical Jacobian.
%
% Evaluation Criteria:
%   - Compares analytical 4x4 Jacobian against central finite difference
%     numerical derivatives with step delta = 1.0e-6 rad.
%   - Relative Frobenius matrix error must be < 1.0e-5 across multiple configurations.
%   - Verifies closed-form determinant against numerical det(J).
%   - Strictly NO Peter Corke Robotics Toolbox. Pure native MATLAB.

fprintf('================================================================================\n');
fprintf('  MILESTONE 2: JACOBIAN NUMERICAL GRADIENT VERIFICATION TEST\n');
fprintf('  Robot: Elephant Robotics myPalletizer 260 Pi (4-DOF)\n');
fprintf('================================================================================\n\n');

rng(101); % Fixed seed for deterministic repeatability

N_configs = 15;
delta = 1.0e-6; % Finite difference perturbation step (rad)
tolerance_rel = 1.0e-5; % Maximum relative error threshold (< 10^-5)

% Generate diverse configurations across the workspace interior
th1_range = deg2rad([-120.0, 120.0]);
th2_range = deg2rad([  10.0,  75.0]);
th3_range = deg2rad([ -70.0,  40.0]); % Avoid exact 0 (singularity)
th4_range = deg2rad([-120.0, 120.0]);

configs = zeros(N_configs, 4);
configs(1, :) = deg2rad([  0.0, 30.0, -45.0,  15.0]);
configs(2, :) = deg2rad([ 45.0, 45.0, -30.0, -15.0]);
configs(3, :) = deg2rad([-60.0, 60.0, -50.0,  30.0]);

for i = 4:N_configs
    configs(i, 1) = th1_range(1) + (th1_range(2) - th1_range(1)) * rand();
    configs(i, 2) = th2_range(1) + (th2_range(2) - th2_range(1)) * rand();
    % Ensure theta3 is not too close to zero to avoid near-singularity
    th3_val = th3_range(1) + (th3_range(2) - th3_range(1)) * rand();
    if abs(th3_val) < 0.05
        th3_val = 0.15 * sign(th3_val + 1e-3);
    end
    configs(i, 3) = th3_val;
    configs(i, 4) = th4_range(1) + (th4_range(2) - th4_range(1)) * rand();
end

fprintf('%-5s | %-16s | %-16s | %-14s | %-14s | %-6s\n', ...
    'Test#', 'Max Abs Err', 'Rel Frob Err', 'det(J) Error', 'Singular?', 'Status');
fprintf('-----------------------------------------------------------------------------------------\n');

max_rel_error_overall = 0.0;
all_passed = true;

for k = 1:N_configs
    th = configs(k, :);

    % 1. Analytical Jacobian and Determinant
    [J_anal, det_anal, is_sing] = jacobian_matrix(th);

    % 2. Central Finite Difference Numerical Jacobian
    J_num = zeros(4, 4);
    for j = 1:4
        th_plus = th;
        th_minus = th;
        th_plus(j)  = th(j) + delta;
        th_minus(j) = th(j) - delta;

        [pose_plus, ~]  = forward_kinematics(th_plus);
        [pose_minus, ~] = forward_kinematics(th_minus);

        % Numerical derivative: dX / dtheta_j
        J_num(:, j) = (pose_plus' - pose_minus') / (2 * delta);
    end

    % 3. Error Metrics
    abs_err_mat = abs(J_anal - J_num);
    max_abs_err = max(abs_err_mat(:));
    rel_frob_err = norm(J_anal - J_num, 'fro') / norm(J_anal, 'fro');

    % 4. Determinant Check: Analytical vs MATLAB det()
    det_num = det(J_anal);
    det_err = abs(det_anal - det_num);

    if rel_frob_err > max_rel_error_overall
        max_rel_error_overall = rel_frob_err;
    end

    status_str = 'PASS';
    if rel_frob_err >= tolerance_rel || det_err > 1e-6
        status_str = 'FAIL';
        all_passed = false;
    end

    fprintf('%-5d | %16.4e | %16.4e | %14.4e | %-14s | %-6s\n', ...
        k, max_abs_err, rel_frob_err, det_err, mat2str(is_sing), status_str);
end

fprintf('-----------------------------------------------------------------------------------------\n');
fprintf('Total Configurations Tested : %d\n', N_configs);
fprintf('Max Relative Frobenius Error: %.4e (Threshold: < 1.0000e-05)\n', max_rel_error_overall);

if all_passed && max_rel_error_overall < tolerance_rel
    fprintf('\n>>> RESULT: ALL %d JACOBIAN GRADIENT TESTS PASSED PERFECTLY <<<\n', N_configs);
else
    error('TEST FAILED: Jacobian relative error exceeded 1.0e-5 threshold!');
end
