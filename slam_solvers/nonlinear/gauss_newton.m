% GAUSS_NEWTON
% 16-833 Spring 2019 - Entire function provided
% Computes a Gauss-Newton update step given the current state estimate
%
% Arguments: 
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
%     x       - new state estimate after Gauss-Newton update
%
function x = gauss_newton(x0, odom, obs, sigma_o, sigma_l)

% GN constants
max_iterations = 10;
epsilon1 = 1E-3;
epsilon2 = 1E-2;

x = x0;
iter = 1;

while (true)
    [A, b] = create_Ab_nonlinear(x, odom, obs, sigma_o, sigma_l);
    delta = solve_linear_system(A, b);
    x_new = x + delta;
    err_old = error_nonlinear(x, odom, obs, sigma_o, sigma_l);
    err_new = error_nonlinear(x_new, odom, obs, sigma_o, sigma_l);
    err_delta = err_new - err_old;
%     fprintf('Norm update: %f\tDelta error: %f\tNew error: %f\tOld error: %f\n', norm(delta), err_delta, err_new, err_old);

    %% Update x if it's a good step, otherwise break
    if (err_delta < 0)
        x = x_new;
    else
        fprintf('Error went up! Skipping this step...\n');
        break;
    end
    
    %% Check end conditions
    if ( norm(delta) < epsilon1 )
%         fprintf('Update got too small at iter %d: %f\n', iter, norm(delta));
        break;
    end
    if ( norm(err_delta) < epsilon2 )
%         fprintf('Difference in error got too small at iter %d: %f\n', iter, err_delta);
        break;
    end
    if (iter > max_iterations)
%         fprintf('Reached max number of iterations\n');    
        break;
    end
    
    iter = iter + 1;
end




