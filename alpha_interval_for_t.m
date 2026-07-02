function [is_feasible, alpha_min, alpha_max] = alpha_interval_for_t(h0, n, t)
%ALPHA_INTERVAL_FOR_T Finds alpha values satisfying:
%
%   -t <= h0_i + alpha*n_i <= t
%
% for all i.

    alpha_min = -inf;
    alpha_max =  inf;

    for i = 1:length(h0)

        if abs(n(i)) < 1e-12
            % This wheel is unaffected by alpha
            if abs(h0(i)) > t
                is_feasible = false;
                return;
            else
                continue;
            end
        end

        lower_i = (-t - h0(i)) / n(i);
        upper_i = ( t - h0(i)) / n(i);

        % If n(i) is negative, the interval flips
        lo = min(lower_i, upper_i);
        hi = max(lower_i, upper_i);

        alpha_min = max(alpha_min, lo);
        alpha_max = min(alpha_max, hi);

        if alpha_min > alpha_max
            is_feasible = false;
            return;
        end
    end

    is_feasible = true;

end