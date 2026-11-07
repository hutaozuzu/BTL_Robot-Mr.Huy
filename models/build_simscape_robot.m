function [modelHandle, modelPath] = build_simscape_robot(varargin)
% BUILD_SIMSCAPE_ROBOT Build and configure 3D Simscape Multibody model for myPalletizer 260 Pi.
%
%   [modelHandle, modelPath] = build_simscape_robot()
%   [modelHandle, modelPath] = build_simscape_robot('CloseOnComplete', true)
%
% This script automates:
%   1. Importing sanitized URDF (mypalletizer_260_pi.urdf) via smimport.
%   2. Setting 4 active revolute joints (joint1, joint2, joint3, joint5) to:
%        - MotionActuationMode = 'InputMotion' (prescribed input motion)
%        - TorqueActuationMode = 'ComputedTorque' (dynamically consistent computed torque)
%        - SensePosition = 'on' (joint telemetry)
%   3. Adding 4 Simulink Inports (q1, q2, q3, q4) and Simulink-PS Converters.
%   4. Adding a Transform Sensor to measure end-effector Cartesian [x, y, z].
%   5. Adding Outports: 'ee_pos' [3x1] (Port 1) and 'q_meas' [4x1] (Port 2).
%   6. Saving the production model as 'mypalletizer_simscape.slx'.
%
% Author: Teamwork Worker M1
% Date: 2026-11-07

    % Parse optional inputs
    p = inputParser;
    addParameter(p, 'CloseOnComplete', false, @islogical);
    addParameter(p, 'ModelName', 'mypalletizer_simscape', @ischar);
    parse(p, varargin{:});
    closeOnComplete = p.Results.CloseOnComplete;
    modelName = p.Results.ModelName;

    % Locate directories
    scriptDir = fileparts(mfilename('fullpath'));
    if isempty(scriptDir)
        scriptDir = pwd;
    end
    projectRoot = fileparts(scriptDir);
    
    meshesDir = fullfile(scriptDir, 'meshes');
    urdfDir = fullfile(scriptDir, 'urdf');
    urdfFile = fullfile(urdfDir, 'mypalletizer_260_pi.urdf');
    
    if ~exist(urdfFile, 'file')
        error('build_simscape_robot:URDFNotFound', 'URDF file not found at: %s', urdfFile);
    end
    
    % Ensure models and meshes are on MATLAB search path
    addpath(scriptDir);
    addpath(meshesDir);
    addpath(urdfDir);
    if exist(projectRoot, 'dir')
        addpath(projectRoot);
    end
    
    fprintf('=== Building Simscape Multibody Robot: %s ===\n', modelName);
    fprintf('  URDF File: %s\n', urdfFile);
    fprintf('  Meshes:    %s\n', meshesDir);
    
    % Close system if already open
    if bdIsLoaded(modelName)
        fprintf('  Closing existing model in memory...\n');
        close_system(modelName, 0);
    end
    
    % Save target .slx path
    targetSlx = fullfile(scriptDir, [modelName, '.slx']);
    
    % Change working directory to models dir during import for relative path integrity
    origDir = pwd;
    cd(scriptDir);
    cleanupDir = onCleanup(@() cd(origDir));
    
    % Run smimport
    fprintf('  Executing smimport...\n');
    [hModel, ~] = smimport('urdf/mypalletizer_260_pi.urdf', 'ModelName', modelName);
    fprintf('  smimport succeeded with model handle: %s\n', num2str(hModel));
    
    % Define active revolute joints and names
    jointBlocks = {'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2', 'joint5_to_joint4'};
    jointLabels = {'q1', 'q2', 'q3', 'q4'};
    
    % Add Mux for measured joint angles
    muxQ = add_block('simulink/Signal Routing/Mux', [modelName, '/Mux_q'], ...
        'Position', [980, 220, 985, 380], ...
        'Inputs', '4');
    
    fprintf('  Configuring active joint actuation, D-H coordinate adapters, and input/output ports...\n');
    % Standard D-H to URDF joint coordinate transformation:
    %   q1_urdf = +1 * q1_dh + 0
    %   q2_urdf = -1 * q2_dh + pi/2
    %   q3_urdf = -1 * q3_dh - pi/2
    %   q4_urdf = +1 * q4_dh + 0
    dhGains  = {'1', '-1', '-1', '1'};
    dhBiases = {'0', 'pi/2', '-pi/2', '0'};

    for i = 1:4
        jPath = [modelName, '/', jointBlocks{i}];
        jName = jointLabels{i};
        yOffset = 80 + (i - 1) * 80;
        
        % 1. Set Actuation & Sensing
        set_param(jPath, 'MotionActuationMode', 'InputMotion');
        set_param(jPath, 'TorqueActuationMode', 'ComputedTorque');
        set_param(jPath, 'SensePosition', 'on');
        
        % 2. Inport block (accepts D-H joint angle in rad)
        inBlk = add_block('simulink/Sources/In1', [modelName, '/', jName], ...
            'Position', [20, yOffset, 50, yOffset + 14], ...
            'Port', num2str(i));
        
        % 2b. D-H to URDF Gain & Bias blocks
        gainInBlk = add_block('simulink/Math Operations/Gain', [modelName, '/dh_gain_in_', jName], ...
            'Position', [70, yOffset - 3, 95, yOffset + 17], ...
            'Gain', dhGains{i});
        biasInBlk = add_block('simulink/Math Operations/Bias', [modelName, '/dh_bias_in_', jName], ...
            'Position', [110, yOffset - 3, 140, yOffset + 17], ...
            'Bias', dhBiases{i});
        
        % 3. Simulink-PS Converter
        s2psBlk = add_block('nesl_utility/Simulink-PS Converter', [modelName, '/s2ps_', jName], ...
            'Position', [160, yOffset - 5, 200, yOffset + 25], ...
            'Unit', 'rad', ...
            'FilteringAndDerivatives', 'filter', ...
            'SimscapeFilterOrder', '2', ...
            'InputFilterTimeConstant', '0.001');
        
        % Connect Inport -> Gain -> Bias -> S2PS -> Joint Motion Input (LConn2)
        add_line(modelName, [jName, '/1'], ['dh_gain_in_', jName, '/1'], 'autorouting', 'smart');
        add_line(modelName, ['dh_gain_in_', jName, '/1'], ['dh_bias_in_', jName, '/1'], 'autorouting', 'smart');
        add_line(modelName, ['dh_bias_in_', jName, '/1'], ['s2ps_', jName, '/1'], 'autorouting', 'smart');
        add_line(modelName, ['s2ps_', jName, '/RConn1'], [jointBlocks{i}, '/LConn2'], 'autorouting', 'smart');
        
        % 4. Joint Position Sensing (RConn2) -> PS2S -> URDF-to-DH Gain & Bias -> Mux
        ps2sBlk = add_block('nesl_utility/PS-Simulink Converter', [modelName, '/ps2s_', jName], ...
            'Position', [840, yOffset + 140, 880, yOffset + 170]);
        gainOutBlk = add_block('simulink/Math Operations/Gain', [modelName, '/dh_gain_out_', jName], ...
            'Position', [900, yOffset + 145, 925, yOffset + 165], ...
            'Gain', dhGains{i});
        biasOutBlk = add_block('simulink/Math Operations/Bias', [modelName, '/dh_bias_out_', jName], ...
            'Position', [940, yOffset + 145, 965, yOffset + 165], ...
            'Bias', dhBiases{i});
        add_line(modelName, [jointBlocks{i}, '/RConn2'], ['ps2s_', jName, '/LConn1'], 'autorouting', 'smart');
        add_line(modelName, ['ps2s_', jName, '/1'], ['dh_gain_out_', jName, '/1'], 'autorouting', 'smart');
        add_line(modelName, ['dh_gain_out_', jName, '/1'], ['dh_bias_out_', jName, '/1'], 'autorouting', 'smart');
        add_line(modelName, ['dh_bias_out_', jName, '/1'], ['Mux_q/', num2str(i)], 'autorouting', 'smart');
    end
    
    % Outport 2: Measured joint angles [4x1]
    outQ = add_block('simulink/Sinks/Out1', [modelName, '/q_meas'], ...
        'Position', [1020, 290, 1050, 304], ...
        'Port', '2');
    add_line(modelName, 'Mux_q/1', 'q_meas/1', 'autorouting', 'smart');
    
    % 5. Add TCP Rigid Transform (a4 = 0.044 m along +X) & Transform Sensor for End-Effector [x, y, z]
    fprintf('  Configuring TCP Offset (a4 = 0.044 m) and End-Effector Transform Sensor [x, y, z]...\n');
    tcpTx = add_block('sm_lib/Frames and Transforms/Rigid Transform', [modelName, '/TCP_Offset'], ...
        'Position', [730, 110, 770, 150], ...
        'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', '[0.044 0 0]');

    ts = add_block('sm_lib/Frames and Transforms/Transform Sensor', [modelName, '/EE_Transform_Sensor'], ...
        'Position', [800, 100, 840, 140], ...
        'SenseXYZ', 'on');
    
    ps2sEE = add_block('nesl_utility/PS-Simulink Converter', [modelName, '/PS_Conv_EE'], ...
        'Position', [880, 105, 920, 135]);
    
    outEE = add_block('simulink/Sinks/Out1', [modelName, '/ee_pos'], ...
        'Position', [980, 110, 1010, 124], ...
        'Port', '1');
    
    % Connect Transform Sensor:
    % Base frame (LConn1) -> World (RConn1)
    % Follower frame (RConn1) -> TCP_Offset (RConn1) <- link5 (LConn1)
    % Translation signal (RConn2) -> PS-Simulink Converter -> Outport ee_pos
    add_line(modelName, 'World/RConn1', 'EE_Transform_Sensor/LConn1', 'autorouting', 'smart');
    add_line(modelName, 'link5/LConn1', 'TCP_Offset/LConn1', 'autorouting', 'smart');
    add_line(modelName, 'TCP_Offset/RConn1', 'EE_Transform_Sensor/RConn1', 'autorouting', 'smart');
    add_line(modelName, 'EE_Transform_Sensor/RConn2', 'PS_Conv_EE/LConn1', 'autorouting', 'smart');
    add_line(modelName, 'PS_Conv_EE/1', 'ee_pos/1', 'autorouting', 'smart');
    
    % Configure Model Solver & Simulation Settings
    set_param(modelName, 'SolverType', 'Variable-step');
    set_param(modelName, 'Solver', 'daessc');
    set_param(modelName, 'MaxStep', '0.005');
    set_param(modelName, 'StopTime', '0.5');
    
    % Save Model
    fprintf('  Saving model to: %s\n', targetSlx);
    save_system(modelName, targetSlx);
    
    modelHandle = hModel;
    modelPath = targetSlx;
    
    if closeOnComplete
        close_system(modelName, 0);
        fprintf('  Model closed.\n');
    end
    
    fprintf('=== Simscape Multibody Robot Build Complete! ===\n\n');
end
