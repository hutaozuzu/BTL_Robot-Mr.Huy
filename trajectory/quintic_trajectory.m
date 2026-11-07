function [s, ds, dds] = quintic_trajectory(s0, sf, t0, tf, t, v0, vf, a0, af)
% QUINTIC_TRAJECTORY Generates 5th-order polynomial trajectory with C^2 continuity.
%
% Computes position s(t), velocity ds(t), and acceleration dds(t) across
% time interval [t0, tf] ensuring boundary conditions:
%   s(t0) = s0,  s(tf) = sf
%   ds(t0) = v0, ds(tf) = vf  (default: 0)
%   dds(t0) = a0, dds(tf) = af (default: 0)
%
% Syntax:
%   [s, ds, dds] = quintic_trajectory(s0, sf, t0, tf, t)
%   [s, ds, dds] = quintic_trajectory(s0, sf, t0, tf, t, v0, vf, a0, af)
%
% Inputs:
%   s0 - Initial position (scalar or [1xD] / [Dx1] vector)
%   sf - Final position (scalar or [1xD] / [Dx1] vector)
%   t0 - Initial time (scalar, seconds)
%   tf - Final time (scalar, seconds, must satisfy tf > t0)
%   t  - Query time instant(s) (scalar, row [1xM], or column [Mx1] vector)
%   v0 - Optional initial velocity (default: 0)
%   vf - Optional final velocity (default: 0)
%   a0 - Optional initial acceleration (default: 0)
%   af - Optional final acceleration (default: 0)
%
% Outputs:
%   s   - Interpolated position at time t
%   ds  - Interpolated velocity at time t
%   dds - Interpolated acceleration at time t
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    % Validate duration
    if nargin < 5
        error('quintic_trajectory:TooFewInputs', 'Must provide s0, sf, t0, tf, and t.');
    end

    T = tf - t0;
    if T <= 0
        error('quintic_trajectory:InvalidTime', ...
              'Final time tf (%.4f) must be strictly greater than initial time t0 (%.4f).', tf, t0);
    end

    % Dimensionality of targets
    num_dim = numel(s0);
    if numel(sf) ~= num_dim
        error('quintic_trajectory:DimensionMismatch', 's0 and sf must have the same number of elements.');
    end

    % Reshape targets to row vectors [1 x D]
    s0_row = reshape(s0, 1, num_dim);
    sf_row = reshape(sf, 1, num_dim);

    % Default boundary velocities and accelerations
    if nargin < 6 || isempty(v0), v0 = zeros(1, num_dim); else, v0 = reshape(v0, 1, num_dim); end
    if nargin < 7 || isempty(vf), vf = zeros(1, num_dim); else, vf = reshape(vf, 1, num_dim); end
    if nargin < 8 || isempty(a0), a0 = zeros(1, num_dim); else, a0 = reshape(a0, 1, num_dim); end
    if nargin < 9 || isempty(af), af = zeros(1, num_dim); else, af = reshape(af, 1, num_dim); end

    % Query time vector
    t_was_row = isrow(t);
    t_vec = t(:); % [M x 1] column vector
    M = numel(t_vec);

    % Normalized time tau in [0, 1] with saturation
    tau = (t_vec - t0) / T;
    tau = max(0.0, min(1.0, tau));

    % Solve polynomial coefficients for each dimension:
    % s(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5
    c0 = s0_row;                     % [1 x D]
    c1 = v0 * T;                     % [1 x D]
    c2 = 0.5 * a0 * (T^2);           % [1 x D]

    h  = sf_row - s0_row;            % [1 x D]
    r1 = h - v0 * T - 0.5 * a0 * (T^2);
    r2 = vf * T - v0 * T - a0 * (T^2);
    r3 = af * (T^2) - a0 * (T^2);

    % Analytical solution via inverse Vandermonde matrix
    c3 =  10.0 * r1 - 4.0 * r2 + 0.5 * r3;
    c4 = -15.0 * r1 + 7.0 * r2 - 1.0 * r3;
    c5 =   6.0 * r1 - 3.0 * r2 + 0.5 * r3;

    % Evaluate polynomials
    % Basis vectors [M x 1]
    tau2 = tau .^ 2;
    tau3 = tau .^ 3;
    tau4 = tau .^ 4;
    tau5 = tau .^ 5;

    if num_dim == 1
        % Scalar output
        s_eval   = c0 + c1 .* tau + c2 .* tau2 + c3 .* tau3 + c4 .* tau4 + c5 .* tau5;
        ds_eval  = (1.0 / T)   * (c1 + 2.0 * c2 .* tau + 3.0 * c3 .* tau2 + 4.0 * c4 .* tau3 + 5.0 * c5 .* tau4);
        dds_eval = (1.0 / T^2) * (2.0 * c2 + 6.0 * c3 .* tau + 12.0 * c4 .* tau2 + 20.0 * c5 .* tau3);

        if t_was_row
            s   = reshape(s_eval, 1, M);
            ds  = reshape(ds_eval, 1, M);
            dds = reshape(dds_eval, 1, M);
        else
            s   = reshape(s_eval, M, 1);
            ds  = reshape(ds_eval, M, 1);
            dds = reshape(dds_eval, M, 1);
        end
    else
        % Vector output [M x D]
        s   = tau.^0 * c0 + tau * c1 + tau2 * c2 + tau3 * c3 + tau4 * c4 + tau5 * c5;
        ds  = (1.0 / T)   * (tau.^0 * c1 + 2.0 * tau * c2 + 3.0 * tau2 * c3 + 4.0 * tau3 * c4 + 5.0 * tau4 * c5);
        dds = (1.0 / T^2) * (tau.^0 * (2.0 * c2) + 6.0 * tau * c3 + 12.0 * tau2 * c4 + 20.0 * tau3 * c5);
    end
end
