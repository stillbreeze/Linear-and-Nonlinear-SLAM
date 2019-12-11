% SOLVE_PINV
% 16-833 Spring 2019 - *Stub* Provided
% Solves linear system using pseudo-inverse
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using the
%             pseudo-inverse, which you compute without built-in MATLAB
%             functions
%
function x = solve_pinv(A, b)

x = (inv(A'*A)*A')*b;

end