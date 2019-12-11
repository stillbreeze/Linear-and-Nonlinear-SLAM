% MEAS_ODOM
% 16-833 Spring 2019 - *Stub* Provided
% Simple function to predict a linear odometry measurement between two
% poses
%
% Arguments: 
%     rx1   - robot's previous x position
%     ry1   - robot's previous y position
%     rx2   - robot's next x position
%     ry2   - robot's next y position
%
% Returns:
%     h     - odometry measurement prediction 
%
function h = meas_odom(rx1, ry1, rx2, ry2)

h = [rx2-rx1;ry2-ry1];