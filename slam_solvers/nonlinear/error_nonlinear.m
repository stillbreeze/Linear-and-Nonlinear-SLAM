% ERROR_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the total error of all measurements (odometry and landmark)
% given the current state estimate
%
% Arguments: 
%     x       - Current estimate of the state vector
%     odom    - Matrix that contains the odometry measurements
%               between consecutive poses. Each row corresponds to
%               a measurement. 
%                 odom(:,1) - x-value of odometry measurement
%                 odom(:,2) - y-value of odometry measurement
%     obs     - Matrix that contains the landmark measurements and
%               relevant information. Each row corresponds to a
%               measurement.
%                 obs(:,1) - idx of pose at which measurement was 
%                   made
%                 obs(:,2) - idx of landmark being observed
%                 obs(:,3) - bearing theta of landmark measurement
%                 obs(:,4) - range d of landmark measurement
%     sigma_o - Covariance matrix corresponding to the odometry
%               measurements
%     sigma_l - Covariance matrix corresponding to the landmark
%               measurements
% Returns:
%     err     - total error of all measurements
%
function err = error_nonlinear(x, odom, obs, sigma_odom, sigma_landmark)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);    % landmark measurement dimension

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize error
err = zeros(n_poses+n_landmarks, 1);
b = zeros(M, 1);

for i = 1:n_poses
    if i ~= 1
        e = meas_odom(x(2*i-3), x(2*i-2), x(2*i-1), x(2*i)) - [odom(i-1,1);odom(i-1,2)];
        err(i) = e'*inv(sigma_odom)*e;
        b(2*i-1:2*i) = e;
    else
        err(i) = 0;
        b(i:i+1) = [0;0];
    end
end

l_start_idx = p_dim*n_poses;
for i = 1:n_obs
    p_idx = obs(i,1);
    l_idx = obs(i,2);
    e = meas_landmark(x(p_idx*2-1), x(p_idx*2), x(l_start_idx+l_idx*2-1), x(l_start_idx+l_idx*2)) - [obs(i,3);obs(i,4)];
    e(1) = wrapToPi(e(1));
    err(n_poses+i) = e'*inv(sigma_landmark)*e;
    b(l_start_idx+2*i-1:l_start_idx+2*i) = e;
end
err = sum(err);
% err = sum(b'*b);
