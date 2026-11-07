function [pose, T_all] = forward_kinematics(theta, angle_unit)
% FORWARD_KINEMATICS Computes end-effector pose and intermediate transforms
% for the 4-DOF myPalletizer 260 Pi robot manipulator.
%
% Syntax:
%   [pose, T_all] = forward_kinematics(theta)
%   [pose, T_all] = forward_kinematics(theta, angle_unit)
%
% Inputs:
%   theta      - [1x4] or [4x1] joint angles [theta1, theta2, theta3, theta4]
%   angle_unit - Optional string: 'rad' (default) or 'deg'
%
% Outputs:
%   pose  - [1x4] end-effector pose [x, y, z, phi]
%           x, y, z in mm
%           phi is tool pitch angle (radians by default, or degrees if angle_unit='deg')
%           phi = theta2 + theta3 + theta4
%   T_all - Cell array {T01, T02, T03, T04} of 4x4 homogeneous transformation
%           matrices from base frame {0} to each joint frame {i}.
%
% Standard D-H Parameters (myPalletizer 260 Pi):
%   Link 1: theta1, d1 = 120.0 mm, a1 = 0.0 mm,   alpha1 = +pi/2
%   Link 2: theta2, d2 = 0.0 mm,   a2 = 140.0 mm, alpha2 = 0.0
%   Link 3: theta3, d3 = 0.0 mm,   a3 = 134.0 mm, alpha3 = 0.0
%   Link 4: theta4, d4 = 0.0 mm,   a4 = 44.0 mm,  alpha4 = 0.0
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    if nargin < 2 || isempty(angle_unit)
        angle_unit = 'rad';
    end

    % Ensure row vector of length 4
    theta = reshape(theta, 1, []);
    if numel(theta) ~= 4
        error('forward_kinematics:InvalidInput', 'theta must contain exactly 4 joint angles.');
    end

    is_deg = strcmpi(angle_unit, 'deg');
    if is_deg
        th_rad = deg2rad(theta);
    else
        th_rad = theta;
    end

    th1 = th_rad(1);
    th2 = th_rad(2);
    th3 = th_rad(3);
    th4 = th_rad(4);

    % Link parameters (mm and rad)
    d1 = 120.0;
    a2 = 140.0;
    a3 = 134.0;
    a4 = 44.0;

    % Individual D-H homogeneous transformation matrices
    A1 = dh_transform(th1, d1, 0.0, pi/2);
    A2 = dh_transform(th2, 0.0, a2, 0.0);
    A3 = dh_transform(th3, 0.0, a3, 0.0);
    A4 = dh_transform(th4, 0.0, a4, 0.0);

    % Intermediate transformations relative to base frame {0}
    T01 = A1;
    T02 = T01 * A2;
    T03 = T02 * A3;
    T04 = T03 * A4;

    T_all = {T01, T02, T03, T04};

    % Tool Center Point (TCP) position from T04
    x = T04(1, 4);
    y = T04(2, 4);
    z = T04(3, 4);

    % Tool pitch angle in arm plane: phi = theta2 + theta3 + theta4
    phi = th2 + th3 + th4;
    if is_deg
        phi = rad2deg(phi);
    end

    pose = [x, y, z, phi];
end
