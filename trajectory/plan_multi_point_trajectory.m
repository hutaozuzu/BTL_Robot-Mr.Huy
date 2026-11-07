function traj_struct = plan_multi_point_trajectory(waypoints, time_per_segment, dt)
% PLAN_MULTI_POINT_TRAJECTORY Interface contract wrapper for multi_point_planner.
%
% Implements the PROJECT.md interface contract:
%   plan_multi_point_trajectory(waypoints, time_per_segment, dt)
%
% Inputs:
%   waypoints        - [Nx4] or [Nx5] matrix of Cartesian [x, y, z, phi] or joint waypoints
%   time_per_segment - [(N-1)x1] or scalar segment duration vector (s)
%   dt               - Sampling step (s, default: 0.02)
%
% Outputs:
%   traj_struct      - Struct with t, theta, dtheta, ddtheta, pose, xyz_actual, is_valid
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    if nargin < 2, time_per_segment = []; end
    if nargin < 3, dt = 0.02; end

    traj_struct = multi_point_planner(waypoints, time_per_segment, dt);
end
