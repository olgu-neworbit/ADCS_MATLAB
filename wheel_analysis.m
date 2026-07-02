theta = 26.565;

a = cosd(theta);
d = a;
b = sqrt(1 - a^2);
c = sqrt(1 - d^2);

wheel.axes = [a, -a, 0, 0;
              b,  b, c, c;
              0,  0, d,-d];

wheel.distribution_matrix = 0.5 * [ 1/a, b/(b^2 + c^2),   0;
                                   -1/a, b/(b^2 + c^2),   0;
                                      0, c/(b^2 + c^2), 1/d;
                                      0, c/(b^2 + c^2),-1/d];

D = wheel.distribution_matrix;

% Set this from your reaction wheel datasheet
H_wheel_max = 0.5;   % [Nms], placeholder

% Optional sanity check: axes * distribution_matrix should be identity
check = wheel.axes * D;
disp(check)

% Row norms of the allocation matrix
row_norms = vecnorm(D, 2, 2);

% Worst row, i.e. the row that saturates fastest
[max_row_norm, worst_wheel_idx] = max(row_norms);

% Least favorable body momentum magnitude
H_least = H_wheel_max / max_row_norm;

% Least favorable direction
n_least = D(worst_wheel_idx, :)' / max_row_norm;

fprintf('Least favorable storage = %.6f Nms\n', H_least);
fprintf('Worst wheel index = %d\n', worst_wheel_idx);
fprintf('Least favorable direction:\n');
disp(n_least)

tol = 1e-10;

row_norms = vecnorm(D, 2, 2);
max_row_norm = max(row_norms);

worst_idx = find(abs(row_norms - max_row_norm) < tol);

H_least = H_wheel_max / max_row_norm;

n_least = zeros(3, numel(worst_idx));

for k = 1:numel(worst_idx)
    i = worst_idx(k);
    n_least(:, k) = D(i, :)' / row_norms(i);
end

% Include positive and negative directions
n_least_all = [n_least, -n_least];

fprintf('Least favorable storage = %.6f Nms\n', H_least);
fprintf('Least favorable directions are columns of this matrix:\n');
disp(n_least_all)


[H_best_pinv, n_best_pinv, V_pinv] = ...
    max_storage_pseudoinverse(D, H_wheel_max);

fprintf('Pseudoinverse maximum storage = %.6f Nms\n', H_best_pinv);
fprintf('Best pseudoinverse direction:\n');
disp(n_best_pinv)



A = wheel.axes;

H_wheel_max = 0.5;   % replace with your actual wheel momentum storage [Nms]

[H_least, n_least, V, K] = least_favourable_minmax_storage(A, H_wheel_max);
disp('miniminimi')
fprintf('Least favourable minmax storage = %.6f Nms\n', H_least);
fprintf('Least favourable direction:\n');
disp(n_least)

[H_best_minmax, n_best_minmax, V_minmax] = ...
    max_storage_minmax(A, H_wheel_max);

fprintf('Minmax maximum storage = %.6f Nms\n', H_best_minmax);
fprintf('Best minmax direction:\n');
disp(n_best_minmax)

function Hmax = momentum_storage_in_direction(D, H_wheel_max, n)
    n = n(:);
    n = n / norm(n);

    wheel_usage_per_unit_body_momentum = abs(D * n);

    Hmax = H_wheel_max / max(wheel_usage_per_unit_body_momentum);
end

function [H_least, n_least, V, K] = least_favourable_minmax_storage(A, H_wheel_max)

    % A is 3x4 wheel.axes
    % H_wheel_max is single-wheel momentum storage limit [Nms]
    %
    % Returns:
    %   H_least = least favourable body momentum storage [Nms]
    %   n_least = body-frame unit direction where storage is worst
    %   V = polytope vertices
    %   K = convex hull facets

    m = size(A, 2);

    % Generate all wheel saturation combinations.
    % For 4 wheels, this gives 2^4 = 16 corner points.
    S = dec2bin(0:(2^m - 1)) - '0';
    S = 2*S - 1;   % convert 0/1 to -1/+1

    % Each row of V is one body momentum vertex
    V = (A * (H_wheel_max * S')).';

    % Remove duplicate points if any
    V = uniquetol(V, 1e-12, 'ByRows', true);

    % Convex hull of the reachable body momentum set
    K = convhulln(V);

    best_distance = inf;
    best_normal = [NaN; NaN; NaN];

    for i = 1:size(K,1)

        p1 = V(K(i,1), :).';
        p2 = V(K(i,2), :).';
        p3 = V(K(i,3), :).';

        % Normal to this facet
        normal = cross(p2 - p1, p3 - p1);

        if norm(normal) < 1e-12
            continue
        end

        normal = normal / norm(normal);

        % Make normal point toward the facet from the origin
        if dot(normal, p1) < 0
            normal = -normal;
        end

        % Distance from origin to the facet plane
        distance = dot(normal, p1);

        if distance < best_distance
            best_distance = distance;
            best_normal = normal;
        end
    end

    H_least = best_distance;
    n_least = best_normal;
end

function [H_best, n_best, V] = max_storage_pseudoinverse(D, H_wheel_max)

    % D is 4x3 distribution matrix
    % H_wheel_max is single wheel momentum limit [Nms]
    %
    % Feasible set:
    %   -H_wheel_max <= D * H_body <= H_wheel_max

    m = size(D, 1);

    % Convert absolute-value constraints into linear inequalities:
    %   G * H_body <= b
    G = [ D;
         -D];

    b = H_wheel_max * ones(2*m, 1);

    % Vertices occur where 3 independent constraints are active
    combos = nchoosek(1:(2*m), 3);

    V = [];

    tol = 1e-10;

    for k = 1:size(combos, 1)

        idx = combos(k, :);

        G_active = G(idx, :);
        b_active = b(idx);

        % Need 3 independent planes to define a point
        if rank(G_active) < 3
            continue
        end

        H_candidate = G_active \ b_active;

        % Check if candidate satisfies all constraints
        if all(G * H_candidate <= b + tol)
            V = [V; H_candidate.'];
        end
    end

    % Remove duplicate vertices
    V = uniquetol(V, 1e-9, 'ByRows', true);

    % Farthest vertex from origin
    vertex_norms = vecnorm(V, 2, 2);

    [H_best, idx_best] = max(vertex_norms);

    H_best_vector = V(idx_best, :).';

    n_best = H_best_vector / norm(H_best_vector);
end

function [H_best, n_best, V] = max_storage_minmax(A, H_wheel_max)

    % A is 3x4 wheel.axes
    % H_wheel_max is single wheel momentum limit [Nms]
    %
    % Feasible set:
    %   H_body = A * h_wheels
    %   |h_i| <= H_wheel_max

    m = size(A, 2);

    % Generate all wheel saturation corner combinations
    S = dec2bin(0:(2^m - 1)) - '0';
    S = 2*S - 1;   % convert 0/1 to -1/+1

    % Each row of V is one body momentum vertex candidate
    V = (A * (H_wheel_max * S.')).';

    % Remove duplicates if geometry creates repeated points
    V = uniquetol(V, 1e-9, 'ByRows', true);

    % Maximum body momentum magnitude
    vertex_norms = vecnorm(V, 2, 2);

    [H_best, idx_best] = max(vertex_norms);

    H_best_vector = V(idx_best, :).';

    n_best = H_best_vector / norm(H_best_vector);
end