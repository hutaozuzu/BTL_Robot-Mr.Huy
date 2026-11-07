function [J_inv, lambda_used] = dls_inverse(J, lambda)
% DLS_INVERSE Computes Damped Least Squares (Levenberg-Marquardt) pseudoinverse.
%
% Syntax:
%   J_inv = dls_inverse(J)
%   J_inv = dls_inverse(J, lambda)
%   [J_inv, lambda_used] = dls_inverse(J, lambda)
%
% Inputs:
%   J           - [4x4] Jacobian matrix (or [mxn] general matrix)
%   lambda      - Optional scalar damping factor. If omitted or empty, an adaptive
%                 damping factor is automatically scheduled based on manipulability/SVD.
%
% Outputs:
%   J_inv       - DLS pseudoinverse: J' * inv(J * J' + lambda^2 * I)
%   lambda_used - Damping factor actually utilized in calculation
%
% Formula:
%   J^\dagger = J^T * (J * J^T + \lambda^2 * I)^{-1}
%
% Adaptive Scheduling:
%   When lambda is omitted, lambda dynamically transitions from 0 in dexterous
%   regions to lambda_max as the minimum singular value sigma_min approaches 0:
%     lambda^2 = lambda_max^2 * (1 - (sigma_min / epsilon_0)^2) for sigma_min < epsilon_0
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    [m, ~] = size(J);

    if nargin < 2 || isempty(lambda)
        % Adaptive damping calculation
        s = svd(J);
        sigma_min = min(s);

        epsilon_0  = 5.0;     % Boundary threshold for damping activation
        lambda_max = 0.05;    % Maximum damping factor

        if sigma_min < epsilon_0
            lambda_sq = (lambda_max^2) * (1.0 - (sigma_min / epsilon_0)^2);
            lambda = sqrt(max(0.0, lambda_sq));
        else
            lambda = 1e-6;    % Baseline epsilon for numerical stability
        end
    end

    lambda_used = lambda;

    % Compute J^T * (J * J^T + lambda^2 * I)^(-1) via matrix right-division
    A = J * J' + (lambda^2) * eye(m);
    J_inv = J' / A;
end
