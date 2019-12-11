% SLAM_2D_LINEAR
% 16-833 Spring 2019 - *Stub* Provided
% A 2D SLAM algorithm
%
% Arguments: 
%     method  - the least squares method you would like to test: {'pinv', 
%               'chol1', 'chol2', 'qr1', 'qr2'}. Or 'all' (or no argument) 
%               to test all of the methods
%
% Returns:
%     none
%
function slam_2D_linear(method)
%% Load data
close all; clc; 
addpath('../util');

load('../../data/2D_linear.mat');
% load('../../data/2D_linear_loop.mat');

if nargin < 1
    method = 'all';
end

%% Useful constants
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

%% Create sparse A matrix and b vector 
[A, b] = create_Ab_linear(odom, observations, sigma_odom, sigma_landmark);

if strcmp(method, 'all')
    tic;
    x_pinv = solve_pinv(A, b);
    t_pinv = toc;
    [traj_pinv, landmarks_pinv] = format_solution(x_pinv, n_poses, n_landmarks, o_dim, m_dim);
    
    tic;
    [x_chol1, R_chol1] = solve_chol1(A, b);
    t_chol1 = toc;
    [traj_chol1, landmarks_chol1] = format_solution(x_chol1, n_poses, n_landmarks, o_dim, m_dim);
    
    tic;
    [x_chol2, R_chol2] = solve_chol2(A, b);
    t_chol2 = toc;
    [traj_chol2, landmarks_chol2] = format_solution(x_chol2, n_poses, n_landmarks, o_dim, m_dim);
    
    tic;
    [x_qr1, R_qr1] = solve_qr1(A, b);
    t_qr1 = toc;
    [traj_qr1, landmarks_qr1] = format_solution(x_qr1, n_poses, n_landmarks, o_dim, m_dim);

    tic;
    [x_qr2, R_qr2] = solve_qr2(A, b);
    t_qr2 = toc;
    [traj_qr2, landmarks_qr2] = format_solution(x_qr2, n_poses, n_landmarks, o_dim, m_dim);

    % Show timing results
    fprintf('Timing Results\n');
    fprintf('Pinv:  %d sec\n', t_pinv);
    fprintf('Chol1: %d sec\n', t_chol1);
    fprintf('Chol2: %d sec\n', t_chol2);
    fprintf('QR1:   %d sec\n', t_qr1);
    fprintf('QR2:   %d sec\n\n', t_qr2);
    
    % Evaluate all methods
    evaluate_method('Pinv', traj_pinv, landmarks_pinv, odom, gt_traj, gt_landmarks, false);
    evaluate_method('Chol1', traj_chol1, landmarks_chol1, odom, gt_traj, gt_landmarks, false);
    evaluate_method('Chol2', traj_chol2, landmarks_chol2, odom, gt_traj, gt_landmarks, false);
    evaluate_method('QR1', traj_qr1, landmarks_qr1, odom, gt_traj, gt_landmarks, false);
    evaluate_method('QR2', traj_qr2, landmarks_qr2, odom, gt_traj, gt_landmarks, true);

    % Plot sparse matrices from the decompositions
    figure;
    subplot(1,4,1); spy(R_chol1); title('Chol1')
    subplot(1,4,2); spy(R_chol2); title('Chol2')
    subplot(1,4,3); spy(R_qr1); title('QR1')
    subplot(1,4,4); spy(R_qr2); title('QR2')
    
elseif strcmp(method, 'pinv')
    x = solve_pinv(A, b);
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('Pinv', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
elseif strcmp(method, 'chol1')
    [x, ~] = solve_chol1(A, b);
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('Chol1', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
elseif strcmp(method, 'chol2')
    [x, ~] = solve_chol2(A, b);
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('Chol2', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
elseif strcmp(method, 'qr1')
    [x, ~] = solve_qr1(A, b);
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('QR1', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
elseif strcmp(method, 'qr2')
    [x, ~] = solve_qr2(A, b);
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('QR2', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
elseif strcmp(method, 'lsqr')
    [x, ~] = lsqr(A, b, 1e-6, 30, [], [], zeros(N,1));
    [traj, landmarks] = format_solution(x, n_poses, n_landmarks, o_dim, m_dim);
    evaluate_method('LSQR', traj, landmarks, odom, gt_traj, gt_landmarks, true);
    
end

