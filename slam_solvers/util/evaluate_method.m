function evaluate_method(name, est_traj, est_landmarks, odom, gt_traj, gt_landmarks, show_plot)


if (norm(rms(est_traj-gt_traj))) > 0.05
    fprintf('%s solution is too far off!\n', name);
else
    fprintf('%s solution is correct!\n', name);
end

%% Plot trajectory and landmark results
if show_plot
    figure; hold on;
    plot(gt_traj(:,1), gt_traj(:,2), 'k')
    odom_only = cumsum(odom);
    plot(odom_only(:,1), odom_only(:,2), 'r')
    plot(est_traj(1:end,1), est_traj(1:end,2), 'b')

    plot(gt_landmarks(:,1), gt_landmarks(:,2), 'ko')
    plot(est_landmarks(:,1), est_landmarks(:,2), 'bx')

    legend('ground truth', 'pure odometry', 'full SLAM', 'Location', 'best');
    title(sprintf('Results for %s', name));

    rmse_odom = norm(rms([0, 0; odom_only]-gt_traj))
    rmse_slam = norm(rms(est_traj-gt_traj))
end