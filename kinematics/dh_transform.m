function A = dh_transform(theta, d, a, alpha)
% DH_TRANSFORM Standard Denavit-Hartenberg (SDH) transformation matrix.
%
% Syntax:
%   A = dh_transform(theta, d, a, alpha)
%
% Inputs:
%   theta - Joint angle (radians) about Z_{i-1}
%   d     - Link offset (mm) along Z_{i-1}
%   a     - Link length (mm) along X_i
%   alpha - Link twist (radians) about X_i
%
% Output:
%   A     - 4x4 homogeneous transformation matrix T_{i-1}^i
%
% Formula:
%   A = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
%     = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
%        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
%        0,           sin(alpha),             cos(alpha),            d;
%        0,           0,                      0,                     1]
%
% Note: STRICTLY NO PETER CORKE ROBOTICS TOOLBOX. Pure native MATLAB.

    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end
