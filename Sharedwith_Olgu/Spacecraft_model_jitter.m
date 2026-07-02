%% RWA Jitter Analysis — 4-Wheel Pyramid Configuration
% Loads F(t), T(t) and RPMRate(t) from a single-wheel ramp-up test,
% assembles 4 wheels in a pyramid config at constant speeds,
% and computes attitude perturbation of the satellite body.
%
% Disturbance model per wheel:
%   τ_i = r × (F_i · ŵ_i)   ← torque from force (moment arm)
%       +  T_i · ŵ_i          ← torque from wheel moment (direct)
%
% Inputs:  RPMRate.txt, F.txt, T.txt  (single-column, 3,000,001 rows, 1 kHz)
% Outputs: torque, attitude time history, attitude PSD
% -------------------------------------------------------------------------

clc; clear; close all;

%% ── 1. USER PARAMETERS ──────────────────────────────────────────────────

% File paths (edit to match your folder)
rpm_file = 'RPMRate.txt';
f_file   = 'F.txt';
t_file   = 'T.txt';

% Satellite inertia (kg·m²)
Ixx = 16;   Iyy = 20;   Izz = 24;
I_diag = [Ixx, Iyy, Izz];

% Pyramid centre → satellite CoM (m)
r_mount = [0.25, 0.23, 0.11];

% Pyramid tilt angle (spin axis from nadir / z-axis)
beta = deg2rad(14.74); %%%%%%%%%%%%%%%% cubespace has 26.5? maybe a param to vary for investigatiON?

% Wheel speeds (RPM) — one per wheel
wheel_rpm = [1000, 2000, 2500, 2500];

% Simulation duration (s) and sampling
T_sim   = 0.06;         % seconds to simulate (60 ms)
dt      = 0.001;         % must match data sample rate (1 kHz) %% a bit low maybe? can only capture 500hz with this?
n_sim   = round(T_sim / dt);

% Template window extracted from ramp data (±half either side of target)
WINDOW_PTS = 5000;       % 5 s of data per wheel

%% ── 2. LOAD RAMP DATA ───────────────────────────────────────────────────

fprintf('Loading data...\n');
tic
rpm_data = load(rpm_file);   % column vector, Hz
f_data   = load(f_file);     % column vector, N
t_data   = load(t_file);     % column vector, N·m
fprintf('  Loaded %d samples in %.1f s\n', length(rpm_data), toc);

%% ── 3. PYRAMID GEOMETRY ─────────────────────────────────────────────────
% Four wheels at azimuth 0°, 90°, 180°, 270°, all tilted at angle beta
% Spin-axis unit vectors in body frame [x, y, z]:

sb = sin(beta);   cb = cos(beta);

wheel_axes = [ sb,   0,  cb;   % wheel 1  (azimuth   0°)
                0,  sb,  cb;   % wheel 2  (azimuth  90°)
              -sb,   0,  cb;   % wheel 3  (azimuth 180°)
                0, -sb,  cb];  % wheel 4  (azimuth 270°)

%% ── 4. EXTRACT F TEMPLATE FOR EACH WHEEL ────────────────────────────────
% Find segment of ramp data where RPMRate ≈ wheel speed, tile to T_sim

wheel_hz    = wheel_rpm / 60;        % convert RPM → Hz
N_wheels    = length(wheel_rpm);
F_wheels    = zeros(n_sim, N_wheels);
T_wheels    = zeros(n_sim, N_wheels);

rng(42);   % reproducible random shifts
for k = 1:N_wheels
    % Index in ramp data closest to target speed
    [~, idx] = min(abs(rpm_data - wheel_hz(k)));

    % Extract window around that index
    s = max(1, idx - floor(WINDOW_PTS/2));
    e = min(length(f_data), s + WINDOW_PTS - 1);
    tmpl_f = f_data(s:e);
    tmpl_t = t_data(s:e);

    fprintf('  Wheel %d: %4d RPM → RPMRate at match = %.3f Hz | F_rms = %.4e | T_rms = %.4e\n', ...
        k, wheel_rpm(k), rpm_data(idx), rms(tmpl_f), rms(tmpl_t));

    % Tile both templates, same random shift for coherence between F and T
    n_tile  = ceil(n_sim / length(tmpl_f)) + 1;
    shift   = randi(length(tmpl_f));

    tiled_f = circshift(repmat(tmpl_f, n_tile, 1), shift);
    tiled_t = circshift(repmat(tmpl_t, n_tile, 1), shift);

    F_wheels(:, k) = tiled_f(1:n_sim);
    T_wheels(:, k) = tiled_t(1:n_sim);
end

%% ── 4b. PLOT: Force vs Time & Force vs Frequency (full 3000 s ramp) ─────

t_full       = (0:length(f_data)-1)' * dt;   % full time axis (3000 s)
wheel_colors = lines(N_wheels);

% --- Compute FFT of full ramp (needed for both F and T spectra) ---
N_full  = length(f_data);
f_axis  = (0:floor(N_full/2)) / (N_full * dt);   % Hz
F_amp   = abs(fft(f_data)) / N_full * 2;
F_amp   = F_amp(1:floor(N_full/2)+1);
T_amp   = abs(fft(t_data)) / N_full * 2;
T_amp   = T_amp(1:floor(N_full/2)+1);
freq_mask = f_axis <= 200;

% --- Subplots: F vs Time | F vs Freq | T vs Time | T vs Freq ---
figure('Name','Force & Moment Overview','Position',[50 50 1400 800]);

% (1) F vs Time
subplot(2,2,1);
plot(t_full, f_data, 'Color', [0.2 0.4 0.8], 'LineWidth', 0.5);
xlabel('Time (s)', 'FontSize', 11);
ylabel('F (N)', 'FontSize', 11);
title('Force vs Time — Full 3000 s Ramp', 'FontSize', 11, 'FontWeight', 'bold');
grid on;  xlim([0, 3000]);

% (2) F vs Frequency
subplot(2,2,2);
semilogy(f_axis(freq_mask), F_amp(freq_mask), 'Color', [0.2 0.4 0.8], 'LineWidth', 0.8);
hold on;
for k = 1:N_wheels
    xline(wheel_hz(k), '--', sprintf('W%d %d RPM', k, wheel_rpm(k)), ...
          'Color', wheel_colors(k,:), 'LineWidth', 1.0, 'Alpha', 0.8, ...
          'LabelHorizontalAlignment', 'right');
end
hold off;
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('|F| (N)', 'FontSize', 11);
title('Force Spectrum — Full 3000 s Ramp', 'FontSize', 11, 'FontWeight', 'bold');
grid on;  xlim([0, 200]);

% (3) T vs Time
subplot(2,2,3);
plot(t_full, t_data, 'Color', [0.8 0.3 0.1], 'LineWidth', 0.5);
xlabel('Time (s)', 'FontSize', 11);
ylabel('T (N·m)', 'FontSize', 11);
title('Moment vs Time — Full 3000 s Ramp', 'FontSize', 11, 'FontWeight', 'bold');
grid on;  xlim([0, 3000]);

% (4) T vs Frequency
subplot(2,2,4);
semilogy(f_axis(freq_mask), T_amp(freq_mask), 'Color', [0.8 0.3 0.1], 'LineWidth', 0.8);
hold on;
for k = 1:N_wheels
    xline(wheel_hz(k), '--', sprintf('W%d %d RPM', k, wheel_rpm(k)), ...
          'Color', wheel_colors(k,:), 'LineWidth', 1.0, 'Alpha', 0.8, ...
          'LabelHorizontalAlignment', 'right');
end
hold off;
xlabel('Frequency (Hz)', 'FontSize', 11);
ylabel('|T| (N·m)', 'FontSize', 11);
title('Moment Spectrum — Full 3000 s Ramp', 'FontSize', 11, 'FontWeight', 'bold');
grid on;  xlim([0, 200]);

sgtitle('Disturbance Force & Moment — Single Wheel Ramp-Up', ...
        'FontSize', 13, 'FontWeight', 'bold');

t_sim = (0:n_sim-1)' * dt;   % time axis for 60 ms simulation

%% ── 5. COMPUTE NET TORQUE ────────────────────────────────────────────────
% Per wheel:  τ_i = r × (F_i · ŵ_i)   (force via moment arm)
%                 + T_i · ŵ_i           (moment projected onto body axes)

tau = zeros(n_sim, 3);   % [τx, τy, τz]  N·m

for k = 1:N_wheels
    % --- contribution from force (moment arm) ---
    F_vec   = F_wheels(:, k) * wheel_axes(k, :);          % (n_sim × 3)  %% shouldnt it be per tot his
    tau_F   = cross(repmat(r_mount, n_sim, 1), F_vec);     % (n_sim × 3)

    % --- contribution from wheel moment (direct projection) ---
    tau_T   = T_wheels(:, k) * wheel_axes(k, :);           % (n_sim × 3)

    tau = tau + tau_F + tau_T;
end

fprintf('\nTorque RMS (N·m):  τx=%.4e  τy=%.4e  τz=%.4e\n', ...
    rms(tau(:,1)), rms(tau(:,2)), rms(tau(:,3)));

%% ── 6. INTEGRATE FOR ATTITUDE PERTURBATION ───────────────────────────────
% α = τ / I  →  ω = ∫α dt  →  θ = ∫ω dt   (small-angle, open-loop)

alpha     = tau ./ I_diag;                    % angular acceleration (rad/s²)
omega_att = cumsum(alpha) * dt;               % angular velocity     (rad/s)
theta     = cumsum(omega_att) * dt;           % attitude angle       (rad)

% Unit conversion  (pick one by setting UNIT below)
%   'arcsec'  → arcseconds        (1 rad = 206264.806 arcsec)
%   'mas'     → milliarcseconds   (1 rad = 206264806  mas  )
UNIT = 'arcsec';   % ← change to 'mas' if needed

RAD2ARCSEC = 206264.806247;
switch lower(UNIT)
    case 'arcsec'
        theta_plot  = theta * RAD2ARCSEC;
        unit_label  = 'arcsec';
        psd_label   = 'arcsec²/Hz';
    case 'mas'
        theta_plot  = theta * RAD2ARCSEC * 1e3;
        unit_label  = 'mas';
        psd_label   = 'mas²/Hz';
    otherwise
        error('UNIT must be ''arcsec'' or ''mas''');
end

fprintf('Attitude peak-peak (%s):  θx=%.4f  θy=%.4f  θz=%.4f\n', ...
    unit_label, ...
    peak2peak(theta_plot(:,1)), ...
    peak2peak(theta_plot(:,2)), ...
    peak2peak(theta_plot(:,3)));

%% ── 7. PLOTS ─────────────────────────────────────────────────────────────

axis_labels = {'x', 'y', 'z'};
colors = lines(3);

% --- Figure 1: Net Disturbance Torque vs Time ---
figure('Name','Torque vs Time','Position',[100 100 1200 700]);
for j = 1:3
    subplot(3,1,j);
    plot(t_sim, tau(:,j), 'Color', colors(j,:), 'LineWidth', 0.5);
    ylabel(sprintf('\\tau_%s (N·m)', axis_labels{j}), 'FontSize', 11);
    title(sprintf('\\tau_%s  |  RMS = %.4e N·m', axis_labels{j}, rms(tau(:,j))), ...
          'FontSize', 10);
    grid on;
    xlim([0, T_sim]);
end
xlabel('Time (s)', 'FontSize', 12);
sgtitle('Net Disturbance Torque — 4 RWs Pyramid Configuration', ...
        'FontSize', 13, 'FontWeight', 'bold');

% --- Figure 2: Attitude Perturbation vs Time ---
figure('Name','Attitude vs Time','Position',[100 100 1200 700]);
for j = 1:3
    subplot(3,1,j);
    plot(t_sim, theta_plot(:,j), 'Color', colors(j,:), 'LineWidth', 0.5);
    ylabel(sprintf('\\theta_%s (%s)', axis_labels{j}, unit_label), 'FontSize', 11);
    title(sprintf('\\theta_%s  |  RMS = %.4f %s', axis_labels{j}, ...
          rms(theta_plot(:,j)), unit_label), 'FontSize', 10);
    grid on;
    xlim([0, T_sim]);
end
xlabel('Time (s)', 'FontSize', 12);
sgtitle({'Satellite Attitude Perturbation — Open-Loop Jitter', ...
         '(1000 / 2000 / 2500 / 2500 RPM)'}, ...
        'FontSize', 13, 'FontWeight', 'bold');

% --- Figure 3: Attitude PSD ---
figure('Name','Attitude PSD','Position',[100 100 1200 550]);
hold on;
legend_entries = {};
for j = 1:3
    win_len = min(4096, n_sim);   % window cannot exceed signal length
    [psd, f_psd] = pwelch(theta_plot(:,j), win_len, [], [], 1/dt);
    mask = f_psd <= 200;
    semilogy(f_psd(mask), psd(mask), 'Color', colors(j,:), 'LineWidth', 0.9);
    legend_entries{end+1} = sprintf('\\theta_%s', axis_labels{j}); %#ok<SAGROW>
end

% Mark wheel fundamental frequencies
wc = {'m','c','r','b'};
for k = 1:N_wheels
    xline(wheel_hz(k), '--', sprintf('W%d %d RPM', k, wheel_rpm(k)), ...
          'Color', wc{k}, 'LineWidth', 0.8, 'Alpha', 0.7, ...
          'LabelHorizontalAlignment', 'right');
end

hold off;
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel(sprintf('PSD (%s)', psd_label), 'FontSize', 12);
title('Attitude Jitter PSD — 4 RWs Pyramid Configuration', ...
      'FontSize', 13, 'FontWeight', 'bold');
legend(legend_entries, 'Location', 'northeast', 'FontSize', 10);
grid on;
set(gca, 'YScale', 'log');
xlim([0, 200]);

fprintf('\nDone. Three figures generated.\n');