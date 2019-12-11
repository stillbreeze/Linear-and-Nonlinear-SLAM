% MEAS_LANDMARK_JACOBIAN
% 16-833 Spring 2019 - *Stub* Provided
% Compute the Jacobian of the measurement function
%
% Arguments: 
%     rx    - robot's x position
%     ry    - robot's y position
%     lx    - landmark's x position
%     ly    - landmark's y position
%
% Returns:
%     H     - Jacobian of the measurement fuction
%
function H = meas_landmark_jacobian(rx, ry, lx, ly)

x = lx-rx;
y = ly-ry;
d = (x^2+y^2)^(1/2);
H = [y/d^2, -x/d^2, -y/d^2, x/d^2; -x/d, -y/d, x/d, y/d];