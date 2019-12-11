% SLAM_2D_NONLINEAR
% 16-833 Spring 2019 - *Stub* Provided
% A 2D SLAM algorithm
%
% Arguments: 
%     none
%
% Returns:
%     none
%
function slam_2D_nonlinear()
%% Load data
close all; clc; 
addpath('../util');
load('../../data/2D_nonlinear.mat');

%% Extract useful info
n_poses = size(gt_traj, 1);
n_landmarks = size(gt_landmarks, 1);
n_odom = size(odom, 1);
n_obs  = size(observations, 1);

p_dim = size(gt_traj, 2);
l_dim = size(gt_landmarks, 2);
o_dim = size(odom, 2);
m_dim = size(observations(1, 3:end), 2);

% A matrix is MxN, b is Nx1
N = p_dim*n_poses + l_dim*n_landmarks;
M = o_dim*(n_odom+1) + m_dim*n_obs;     % +1 for prior on the first pose

%% Solve the SLAM problem at each step

% Book-keeping
poses = [0; 0];
all_landmarks = nan(n_landmarks, l_dim);
n_seen = 0;

for i = 1:n_poses
    tps = (i-1)*p_dim+1;
    tpe = i*p_dim;
    lps = (i-2)*p_dim+1;
    lpe = (i-1)*p_dim;
    if (i > 1)
        % Update pose with odometry
        poses(tps:tpe) = poses(lps:lpe) + odom(i-1, :)';
    end
    
    %%%% Add new landmarks %%%%
    obs = observations(observations(:,1) <= i, :);
    new_obs = obs(obs(:,1) == i, :);
    for j = 1:size(new_obs,1)
        landmark_idx = new_obs(j, 2);
        if isnan(all_landmarks(landmark_idx, 1))
            all_landmarks(landmark_idx, :) = project_br_measurement(poses(tps:tpe), new_obs(j, 3:end));
            n_seen = n_seen + 1;
        end
    end
    
    %%%% Re-index landmarks & combine into vector %%%%
    num = 1;
    landmark_map = nan(n_seen, 1);
    landmark_vec = nan(n_seen, 1);
    for j = 1:n_landmarks
        if ~isnan(all_landmarks(j, 1))
            obs(obs(:,2) == j, 2) = num;
            landmark_vec(l_dim*(num-1)+1:l_dim*num) = all_landmarks(j,:);
            landmark_map(num) = j;
            num = num + 1;
        end
    end
    
    x0 = [poses; landmark_vec];
   
    %%%% Update the solution using Gauss-Newton algorithm %%%%
    if i > 1
        x = gauss_newton(x0, odom(1:i-1,:), obs, sigma_odom, sigma_landmark);
    else
        x = x0;
    end

    [traj, landmarks] = format_solution(x, i, n_seen, o_dim, m_dim);
    update_plot('Nonlinear SLAM', traj, landmarks, odom(1:i-1,:), gt_traj(1:i,:), gt_landmarks);
    pause(0.01);
    
    %%%% Update poses and global landmarks %%%%
    for j = 1:i
        poses = x(1:i*p_dim);
    end
    
    updated_landmarks = x(length(poses)+1:end);
    for j = 1:n_seen
        landmark_idx = landmark_map(j);
        all_landmarks(landmark_idx, :) = updated_landmarks(l_dim*(j-1)+1:l_dim*j);
    end
end


evaluate_method('Nonlinear SLAM', traj, landmarks, odom, gt_traj, gt_landmarks, true);

