%% find_worst_axis_nozero_storage.m
clear; clc;

%% Wheel geometry
a = cosd(26.5);
d = a;
b = sqrt(1 - a^2);
c = sqrt(1 - d^2);

A = [a, -a,  0,  0;
     b,  b,  c,  c;
     0,  0,  d, -d];

D = 0.5 * [ 1/a, b/(b^2 + c^2),   0;
           -1/a, b/(b^2 + c^2),   0;
              0, c/(b^2 + c^2), 1/d;
              0, c/(b^2 + c^2),-1/d];

%% Constraints
h_bias = [0.25; 0.25; -0.25; -0.25];
hmax = 0.5;

% Use zero_margin = 0 for theoretical boundary.
% Use something like 1e-4 or 1e-3 if you require strictly no zero-touching.
zero_margin = 0;

s = sign(h_bias);

lb = zeros(4,1);
ub = zeros(4,1);

for i = 1:4
    if s(i) > 0
        lb(i) = zero_margin;
        ub(i) = hmax;
    else
        lb(i) = -hmax;
        ub(i) = -zero_margin;
    end
end

%% Null-space vector
n = null(A);
n = n(:,1);

% Align null vector with the starting bias direction
if dot(n, h_bias) < 0
    n = -n;
end

n = n / norm(n);

fprintf('A*h_bias = \n');
disp(A*h_bias);

fprintf('Null vector n = \n');
disp(n);

%% Search over body axes
Ngrid = 12000;
U = fibonacciSphere(Ngrid);

capacity = zeros(Ngrid,1);

for k = 1:Ngrid
    capacity(k) = capacityAlongAxis(U(:,k), A, D, h_bias, n, lb, ub);
end

[~, idx] = min(capacity);
u0 = U(:,idx);

%% Refine with fminsearch
opts = optimset('Display','off', ...
                'TolX',1e-12, ...
                'TolFun',1e-12, ...
                'MaxIter',2000, ...
                'MaxFunEvals',10000);

obj = @(q) capacityObjective(q, A, D, h_bias, n, lb, ub);

[q_best, H_min] = fminsearch(obj, u0, opts);

worst_axis = q_best(:) / norm(q_best);

% Flip/sign-standardize only for cleaner display.
% The sign variants are symmetry-equivalent.
worst_axis = abs(worst_axis);

H_min = capacityAlongAxis(worst_axis, A, D, h_bias, n, lb, ub);

[h_boundary, alpha_boundary] = allocateAtMagnitude(H_min, worst_axis, D, h_bias, n, lb, ub);

fprintf('\nWorst-case unit body axis:\n');
disp(worst_axis);

fprintf('Minimum no-zero-crossing body momentum storage:\n');
disp(H_min);

fprintf('Boundary wheel momenta:\n');
disp(h_boundary);

fprintf('Max abs wheel momentum:\n');
disp(max(abs(h_boundary)));

fprintf('Minimum signed zero margin:\n');
disp(min(sign(h_bias).*h_boundary));

fprintf('Null-space alpha at boundary:\n');
disp(alpha_boundary);

fprintf('Check A*h_boundary:\n');
disp(A*h_boundary);

fprintf('Check direction A*h_boundary / norm(A*h_boundary):\n');
disp((A*h_boundary) / norm(A*h_boundary));

%% ---------- Local functions ----------

function val = capacityObjective(q, A, D, h_bias, n, lb, ub)
    q = q(:);

    if norm(q) < 1e-12
        val = 1e9;
        return;
    end

    u = q / norm(q);
    val = capacityAlongAxis(u, A, D, h_bias, n, lb, ub);
end

function H_cap = capacityAlongAxis(u, A, D, h_bias, n, lb, ub)
    u = u(:) / norm(u);

    lo = 0;
    hi = 1;

    % Increase upper bound until infeasible
    while isFeasibleMagnitude(hi, u, D, h_bias, n, lb, ub)
        hi = 2*hi;

        if hi > 1e6
            error('Upper bound grew too large. Check geometry/constraints.');
        end
    end

    % Bisection
    for k = 1:80
        mid = 0.5*(lo + hi);

        if isFeasibleMagnitude(mid, u, D, h_bias, n, lb, ub)
            lo = mid;
        else
            hi = mid;
        end
    end

    H_cap = lo;
end

function ok = isFeasibleMagnitude(Hmag, u, D, h_bias, n, lb, ub)
    h0 = h_bias + D*(Hmag*u);

    [ok, ~, ~] = alphaInterval(h0, n, lb, ub);
end

function [h, alpha] = allocateAtMagnitude(Hmag, u, D, h_bias, n, lb, ub)
    h0 = h_bias + D*(Hmag*u);

    [ok, alpha_min, alpha_max] = alphaInterval(h0, n, lb, ub);

    if ~ok
        error('Requested magnitude is infeasible.');
    end

    alpha = 0.5*(alpha_min + alpha_max);
    h = h0 + alpha*n;
end

function [ok, alpha_min, alpha_max] = alphaInterval(h0, n, lb, ub)
    alpha_min = -inf;
    alpha_max =  inf;

    for i = 1:4
        if abs(n(i)) < 1e-12
            if h0(i) < lb(i) || h0(i) > ub(i)
                ok = false;
                return;
            else
                continue;
            end
        end

        a1 = (lb(i) - h0(i)) / n(i);
        a2 = (ub(i) - h0(i)) / n(i);

        lo_i = min(a1, a2);
        hi_i = max(a1, a2);

        alpha_min = max(alpha_min, lo_i);
        alpha_max = min(alpha_max, hi_i);

        if alpha_min > alpha_max
            ok = false;
            return;
        end
    end

    ok = true;
end

function U = fibonacciSphere(N)
    i = (0:N-1)';

    z = 1 - 2*(i + 0.5)/N;
    r = sqrt(max(0, 1 - z.^2));

    golden_angle = pi*(3 - sqrt(5));
    theta = golden_angle*i;

    x = r .* cos(theta);
    y = r .* sin(theta);

    U = [x.'; y.'; z.'];
end