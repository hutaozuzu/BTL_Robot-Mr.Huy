function [J, det_J, is_singular] = jacobian_matrix(theta)
% JACOBIAN_MATRIX Computes analytical 4x4 Jacobian matrix, determinant, and singularity flag.
%
% Syntax:
%   J = jacobian_matrix(theta)
%   [J, det_J, is_singular] = jacobian_matrix(theta)
%
% Inputs:
%   theta       - [1x4] or [4x1] joint angles [theta1, theta2, theta3, theta4] in radians
%
% Outputs:
%   J           - [4x4] Analytical Jacobian matrix relating joint velocities
%                 [dth1, dth2, dth3, dth4]' to task velocities [dx, dy, dz, dphi]'
%   det_J       - Scalar determinant of J: det(J) = -r * a2 * a3 * sin(theta3)
%   is_singular - Logical true if robot is at or near a kinematic singularity (|det_J| < 1e-4)
%
% Singularities:
%   1. Elbow Singularity:    sin(theta3) = 0 (arm fully outstretched or folded)
%   2. Shoulder Singularity: r = 0 (tool center point on base vertical Z0 axis)
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    theta = reshape(theta, 1, []);
    if numel(theta) ~= 4
        error('jacobian_matrix:InvalidInput', 'theta must contain exactly 4 joint angles.');
    end

    th1 = theta(1);
    th2 = theta(2);
    th3 = theta(3);
    th4 = theta(4);

    % Link lengths (mm)
    a2 = 140.0;
    a3 = 134.0;
    a4 = 44.0;

    th23  = th2 + th3;
    th234 = th2 + th3 + th4;

    % Planar radial components and derivatives
    r  = a2 * cos(th2) + a3 * cos(th23) + a4 * cos(th234);
    u2 = -a2 * sin(th2) - a3 * sin(th23) - a4 * sin(th234);
    u3 = -a3 * sin(th23) - a4 * sin(th234);
    u4 = -a4 * sin(th234);

    w2 = r;
    w3 = a3 * cos(th23) + a4 * cos(th234);
    w4 = a4 * cos(th234);

    % Closed-form 4x4 Analytical Jacobian
    J = [-r * sin(th1),  u2 * cos(th1),  u3 * cos(th1),  u4 * cos(th1);
          r * cos(th1),  u2 * sin(th1),  u3 * sin(th1),  u4 * sin(th1);
          0.0,           w2,             w3,             w4;
          0.0,           1.0,            1.0,            1.0];

    % Analytical determinant: det(J) = -r * a2 * a3 * sin(th3)
    det_J = -r * a2 * a3 * sin(th3);

    % Singularity threshold
    singular_tol = 1e-4;
    is_singular = abs(det_J) < singular_tol;
end
