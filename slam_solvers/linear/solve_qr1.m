% SOLVE_QR1
% 16-833 Spring 2019 - *Stub* Provided
% Solves linear system using first QR method
%
% Arguments: 
%     A     - A matrix from linear system that you generate yourself
%     b     - b vector from linear system that you generate yourself
%
% Returns:
%     x     - solution to the linear system, computed using the specified
%             version of the QR decomposition
%     R     - R factor from the QR decomposition
%
function [x, R] = solve_qr1(A, b)

[C,R] = qr(A,b,0);
x = back_sub(R,C);

end