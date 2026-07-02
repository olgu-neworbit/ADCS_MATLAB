function [h, info] = allocate_rw_minmax(H_body, A)
%ALLOCATE_RW_MINMAX Min-max reaction wheel momentum allocation.
%
%   Solves:
%
%       A*h = H_body
%
%   while minimizing:
%
%       max(abs(h_i))
%
%   Inputs:
%       H_body : 3x1 desired body momentum vector
%       A      : 3x4 wheel axes matrix
%
%   Outputs:
%       h      : 4x1 allocated wheel momenta
%       info   : diagnostic struct

    H_body = H_body(:);

    % Check dimensions
    if size(A,1) ~= 3 || size(A,2) ~= 4
        error('A must be a 3x4 wheel axes matrix.');
    end

    if length(H_body) ~= 3
        error('H_body must be a 3x1 vector.');
    end

    % Particular solution using right pseudoinverse.
    % This gives one valid solution to A*h = H_body.
    h_particular = A' * ((A*A') \ H_body);

    % Null-space vector.
    % Any vector n satisfying A*n = 0 can be added without changing body momentum.
    N = null(A);

    if size(N,2) ~= 1
        error('Expected a one-dimensional null space for a 3x4 full-rank wheel matrix.');
    end

    n = N(:,1);

    % Normalize null vector. Scaling does not affect the final h, only alpha.
    n = n / norm(n);

    % Find alpha such that:
    %
    %   h = h_particular + alpha*n
    %
    % minimizes max(abs(h)).
    [alpha_star, h, t_star] = minmax_null_scalar(h_particular, n);

    % Diagnostics
    info.h_particular = h_particular;
    info.null_vector = n;
    info.alpha_star = alpha_star;
    info.max_abs_wheel_momentum = t_star;
    info.body_momentum_error = A*h - H_body;
    info.body_momentum_error_norm = norm(A*h - H_body);

end