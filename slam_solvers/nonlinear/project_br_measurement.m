% PROJECT_BR_MEASUREMENT
% 16-833 Spring 2019 - *Stub* Provided
% Simple function project a bearing range measurement in 2D
%
% Arguments: 
%     pose        - x, y pose
%     measurement - bearing, range measurement
%
% Returns:
%     landmark    - 2D landmark location
%
function landmark = project_br_measurement(pose, measurement)
landmark(1) = pose(1) + measurement(2)*cos(measurement(1));
landmark(2) = pose(2) + measurement(2)*sin(measurement(1));