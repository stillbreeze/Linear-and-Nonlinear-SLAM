% CREATE_AB_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% Computes the A and b matrices for the 2D nonlinear SLAM problem
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
%     A       - MxN matrix
%     b       - Mx1 vector
%
function [As, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l)
%% Extract useful constants which you may wish to use
n_poses = size(odom, 1) + 1;                % +1 for prior on the first pose
n_landmarks = max(obs(:,2));

n_odom = size(odom, 1);
n_obs  = size(obs, 1);

% Dimensions of state variables and measurements (all 2 in this case)
p_dim = 2;                                  % pose dimension
l_dim = 2;                                  % landmark dimension
o_dim = size(odom, 2);                      % odometry dimension
m_dim = size(obs(1, 3:end), 2);             % landmark measurement dimension

% A matrix is MxN, b is Mx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;         % +1 for prior on the first pose

%% Initialize matrices
A = zeros(M, N);
b = zeros(M, 1);

sigma_o_inv = inv(sqrt(sigma_o));
sigma_l_inv = inv(sqrt(sigma_l));

H_o = eye(2);
for i = 1:n_poses
    A(2*i-1:2*i,2*i-1:2*i) = sigma_o_inv*H_o;
    if i ~= 1
        A(2*i-1:2*i,2*i-3:2*i-2) = -sigma_o_inv*H_o;
    end
    
    if i ~= 1
        b(2*i-1:2*i) = sigma_o_inv*([odom(i-1,1);odom(i-1,2)] - meas_odom(x(2*i-3), x(2*i-2), x(2*i-1), x(2*i)));
    else
        b(i:i+1) = [0;0];
    end
end

l_start_idx = p_dim*n_poses;
for i = 1:n_obs
    p_idx = obs(i,1);
    l_idx = obs(i,2);
    H_m = meas_landmark_jacobian(x(2*p_idx-1), x(2*p_idx), x(l_start_idx+2*l_idx-1), x(l_start_idx+2*l_idx));
    A(l_start_idx+2*i-1:l_start_idx+2*i, 2*p_idx-1:2*p_idx) = sigma_l_inv*H_m(:,1:2);
    A(l_start_idx+2*i-1:l_start_idx+2*i, l_start_idx+2*l_idx-1:l_start_idx+2*l_idx) = sigma_l_inv*H_m(:,3:4);
    e = [obs(i,3);obs(i,4)] - meas_landmark(x(p_idx*2-1), x(p_idx*2), x(l_start_idx+l_idx*2-1), x(l_start_idx+l_idx*2));
    e(1) = wrapToPi(e(1));
    b(l_start_idx+2*i-1:l_start_idx+2*i) = sigma_l_inv*e;
end

%% Make A a sparse matrix 
As = sparse(A);
