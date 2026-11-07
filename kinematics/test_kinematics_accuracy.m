% TEST_KINEMATICS_ACCURACY Automated unit test verifying FK-IK round-trip precision.
%
% Evaluation Criteria:
%   - At least 20 random joint configurations within the legal robot workspace.
%   - Forward Kinematics -> Inverse Kinematics -> Reconstructed Forward Kinematics.
%   - Maximum Cartesian position error ||P_original - P_reconstructed|| < 1.0e-4 mm.
%   - Strictly NO Peter Corke Robotics Toolbox. Pure native MATLAB.

fprintf('================================================================================\n');
fprintf('  MILESTONE 2: KINEMATICS ACCURACY UNIT TEST (FK-IK ROUND-TRIP)\n');
fprintf('  Robot: Elephant Robotics myPalletizer 260 Pi (4-DOF)\n');
fprintf('================================================================================\n\n');

rng(42); % Fixed seed for deterministic repeatability

% Number of random test points
N_random = 25;

% Generate test configurations strictly inside the admissible joint limits:
%   Joint 1: [-162, +162] deg -> interior [-150, +150] deg
%   Joint 2: [  -2,  +90] deg -> interior [   5,  +80] deg
%   Joint 3: [ -92,  +60] deg -> interior [ -80,  +50] deg
%   Joint 4: [-180, +180] deg -> interior [-160, +160] deg

th1_range = deg2rad([-150.0, 150.0]);
th2_range = deg2rad([   5.0,  80.0]);
th3_range = deg2rad([ -80.0,  50.0]);
th4_range = deg2rad([-160.0, 160.0]);

% Specific deterministic test cases
fixed_configs = [
    0.0, 0.0, 0.0, 0.0;                                     % Canonical zero pose
    deg2rad([  0.0, 45.0, -45.0,   0.0]);                   % Level arm
    deg2rad([ 45.0, 30.0, -60.0,  30.0]);                   % Palletizing pick
    deg2rad([-45.0, 60.0, -30.0, -30.0]);                   % Palletizing place
    deg2rad([ 90.0, 20.0, -50.0,  30.0]);                   % Sideways reach
];

% Random configurations
random_configs = zeros(N_random, 4);
for i = 1:N_random
    random_configs(i, 1) = th1_range(1) + (th1_range(2) - th1_range(1)) * rand();
    random_configs(i, 2) = th2_range(1) + (th2_range(2) - th2_range(1)) * rand();
    random_configs(i, 3) = th3_range(1) + (th3_range(2) - th3_range(1)) * rand();
    random_configs(i, 4) = th4_range(1) + (th4_range(2) - th4_range(1)) * rand();
end

all_test_configs = [fixed_configs; random_configs];
N_total = size(all_test_configs, 1);

fprintf('Evaluating %d test configurations (5 canonical + %d random interior points)...\n\n', N_total, N_random);
fprintf('%-5s | %-24s | %-24s | %-13s | %-12s | %-6s\n', ...
    'Test#', 'P_orig [x, y, z] mm', 'P_rec [x, y, z] mm', 'Pos Err (mm)', 'Rot Err(rad)', 'Status');
fprintf('----------------------------------------------------------------------------------------------------\n');

max_pos_err = 0.0;
max_rot_err = 0.0;
all_passed = true;
pos_errors = zeros(N_total, 1);

tolerance_pos = 1.0e-4; % 1.0e-4 mm (0.1 micron) required by dispatch
tolerance_rot = 1.0e-6; % 1.0e-6 rad

for k = 1:N_total
    th_orig = all_test_configs(k, :);

    % Forward Kinematics
    [pose_orig, T_all] = forward_kinematics(th_orig);
    P_orig = pose_orig(1:3);
    phi_orig = pose_orig(4);

    % Inverse Kinematics (seeded with th_orig to select closest branch)
    [th_rec, ik_success, all_sols] = inverse_kinematics(pose_orig, 'closest', th_orig);

    if ~ik_success || isempty(th_rec)
        fprintf('%-5d | FAIL: IK solver reported no solution for valid pose!\n', k);
        all_passed = false;
        continue;
    end

    % Reconstructed Forward Kinematics
    [pose_rec, ~] = forward_kinematics(th_rec);
    P_rec = pose_rec(1:3);
    phi_rec = pose_rec(4);

    % Compute Euclidean position and rotation errors
    pos_err = norm(P_orig - P_rec);
    rot_err = abs(atan2(sin(phi_orig - phi_rec), cos(phi_orig - phi_rec)));

    pos_errors(k) = pos_err;
    if pos_err > max_pos_err
        max_pos_err = pos_err;
    end
    if rot_err > max_rot_err
        max_rot_err = rot_err;
    end

    status_str = 'PASS';
    if pos_err >= tolerance_pos || rot_err >= tolerance_rot
        status_str = 'FAIL';
        all_passed = false;
    end

    orig_str = sprintf('[%6.1f, %6.1f, %6.1f]', P_orig(1), P_orig(2), P_orig(3));
    rec_str  = sprintf('[%6.1f, %6.1f, %6.1f]', P_rec(1),  P_rec(2),  P_rec(3));
    fprintf('%-5d | %-24s | %-24s | %13.4e | %12.4e | %-6s\n', ...
        k, orig_str, rec_str, pos_err, rot_err, status_str);
end

fprintf('----------------------------------------------------------------------------------------------------\n');
fprintf('Total Test Points Evaluated : %d\n', N_total);
fprintf('Mean Cartesian Error        : %.4e mm\n', mean(pos_errors));
fprintf('Max Cartesian Error         : %.4e mm (Threshold: < 1.0000e-04 mm)\n', max_pos_err);
fprintf('Max Orientation Error       : %.4e rad\n', max_rot_err);

if all_passed && max_pos_err < tolerance_pos
    fprintf('\n>>> RESULT: ALL %d KINEMATICS ACCURACY TESTS PASSED PERFECTLY <<<\n', N_total);
else
    error('TEST FAILED: Cartesian position error exceeded 1.0e-4 mm threshold!');
end
