x = is2.y0000(:);              % column vector
Ts = 0.005;            % example sampling period, seconds
Fs = 1/Ts;             % sampling frequency, Hz

x = detrend(x, 0);     % remove mean; use detrend(x) if there is slow linear drift

N = length(x);
segLength = min(4096, floor(N/10));
win = hann(segLength, "periodic");
noverlap = round(0.5 * segLength);
nfft = max(2^nextpow2(segLength), segLength);

[pxx, f] = pwelch(x, win, noverlap, nfft, Fs);

figure;
plot(f, 10*log10(pxx));
grid on;
xlabel("Frequency [Hz]");
ylabel("PSD [units^2/Hz]");
title("Welch Power Spectral Density");