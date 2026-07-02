function [alpha_star, h_star, t_star] = minmax_null_scalar(h0, n)
%MINMAX_NULL_SCALAR Solve:
%
%   minimize_alpha max(abs(h0 + alpha*n))
%
% This is a 1D convex min-max problem. We solve it without Optimization
% Toolbox by bisection on t, where:
%
%   abs(h0_i + alpha*n_i) <= t
%
% for every wheel i.

    h0 = h0(:);
    n  = n(:);

    if length(h0) ~= length(n)
        error('h0 and n must have the same length.');
    end

    % alpha = 0 is always feasible with this t
    t_low = 0;
    t_high = max(abs(h0));

    % Bisection
    for k = 1:80
        t_mid = 0.5 * (t_low + t_high);

        [is_feasible, ~, ~] = alpha_interval_for_t(h0, n, t_mid);

        if is_feasible
            t_high = t_mid;
        else
            t_low = t_mid;
        end
    end

    t_star = t_high;

    % Get feasible alpha interval at optimal t
    [is_feasible, alpha_min, alpha_max] = alpha_interval_for_t(h0, n, t_star);

    if ~is_feasible
        error('Internal error: no feasible alpha found.');
    end

    % Pick middle of feasible interval
    alpha_star = 0.5 * (alpha_min + alpha_max);

    h_star = h0 + alpha_star*n;

    % Recompute actual maximum wheel momentum
    t_star = max(abs(h_star));

end