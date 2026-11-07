function [is_valid, details] = check_limits(val, check_type, angle_unit)
% CHECK_LIMITS Validates robot joint angles or Cartesian pose against hardware bounds.
%
% Syntax:
%   is_valid = check_limits(theta)
%   [is_valid, details] = check_limits(val, check_type, angle_unit)
%
% Inputs:
%   val        - [1x4] or [4x1] vector of joint angles or Cartesian pose
%   check_type - Optional: 'joints' (default) or 'cartesian'
%   angle_unit - Optional: 'rad' (default) or 'deg'
%
% Outputs:
%   is_valid - Logical true if within legal limits, false otherwise
%   details  - Struct containing:
%                .in_limits   - Logical array of per-element validity
%                .lower_bounds- Lower limit vector
%                .upper_bounds- Upper limit vector
%                .message     - Diagnostic string
%
% Limits (myPalletizer 260 Pi):
%   Joint 1 (Base):     [-162 deg, +162 deg] = [-2.827433 rad, +2.827433 rad]
%   Joint 2 (Shoulder): [ -2 deg,  +90 deg]  = [-0.034907 rad, +1.570796 rad]
%   Joint 3 (Elbow):    [-92 deg,  +60 deg]  = [-1.605703 rad, +1.047198 rad]
%   Joint 4 (Wrist):    [-180 deg, +180 deg] = [-pi rad,       +pi rad]
%
% Cartesian Limits:
%   X: [-260 mm, +260 mm]
%   Y: [-260 mm, +260 mm]
%   Z: [ -15 mm, +358 mm]
%   R: [   0 mm,  318 mm] (Max physical reach: 140 + 134 + 44 = 318 mm)
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    if nargin < 2 || isempty(check_type)
        check_type = 'joints';
    end
    if nargin < 3 || isempty(angle_unit)
        angle_unit = 'rad';
    end

    val = reshape(val, 1, []);
    tol = 1e-6; % Numerical tolerance for float comparison

    details = struct();

    switch lower(check_type)
        case {'joint', 'joints'}
            if numel(val) ~= 4
                error('check_limits:InvalidInput', 'Joint vector must have 4 elements.');
            end

            is_deg = strcmpi(angle_unit, 'deg');
            if is_deg
                lower_lim = [-162.0,  -2.0, -92.0, -180.0];
                upper_lim = [ 162.0,  90.0,  60.0,  180.0];
                tol_val   = rad2deg(tol);
            else
                lower_lim = deg2rad([-162.0,  -2.0, -92.0, -180.0]);
                upper_lim = deg2rad([ 162.0,  90.0,  60.0,  180.0]);
                tol_val   = tol;
            end

            in_limits = (val >= (lower_lim - tol_val)) & (val <= (upper_lim + tol_val));
            is_valid  = all(in_limits);

            details.in_limits    = in_limits;
            details.lower_bounds = lower_lim;
            details.upper_bounds = upper_lim;

            if is_valid
                details.message = 'All joints within hardware limits.';
            else
                violated = find(~in_limits);
                v_str = sprintf('Joint %d ', violated);
                details.message = sprintf('Limit violation at %s: values=%s', v_str, mat2str(val(violated), 4));
            end

        case {'cartesian', 'cart', 'pose'}
            if numel(val) < 3
                error('check_limits:InvalidInput', 'Cartesian input must have at least [x, y, z].');
            end

            x = val(1);
            y = val(2);
            z = val(3);
            r = sqrt(x^2 + y^2);

            x_ok = (x >= -260.0 - tol) && (x <= 260.0 + tol);
            y_ok = (y >= -260.0 - tol) && (y <= 260.0 + tol);
            z_ok = (z >=  -15.0 - tol) && (z <= 358.0 + tol);
            r_ok = (r <= 318.0 + tol);

            in_limits = [x_ok, y_ok, z_ok, r_ok];
            is_valid  = all(in_limits);

            details.in_limits    = in_limits;
            details.lower_bounds = [-260, -260, -15, 0];
            details.upper_bounds = [ 260,  260, 358, 318];

            if is_valid
                details.message = 'Cartesian position within legal workspace.';
            else
                details.message = sprintf('Cartesian bounds violation: x=%.1f, y=%.1f, z=%.1f, r=%.1f', x, y, z, r);
            end

        otherwise
            error('check_limits:UnknownType', 'Unknown check_type: %s. Use ''joints'' or ''cartesian''.', check_type);
    end
end
