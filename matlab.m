
clc;
clear;
close all;

%% Serial configuration
port = "COM19";
baudRate = 115200;

s = serialport(port, baudRate);
configureTerminator(s, "LF");
flush(s);

%% Configuration
maxSamples = 100000;

NFFT = 1024;
updateEvery = 100;
rawSaveEvery = 100;

rawFile = "pipeline_sensor_data.csv";
fftFile = "pipeline_fft_properties.csv";

%% Initialize CSV files
fid = fopen(rawFile, 'w');
if fid < 0
    clear s;
    error("Cannot create raw CSV file.");
end
fprintf(fid, 'time_us,P1,P2\n');
fclose(fid);

fid = fopen(fftFile, 'w');
if fid < 0
    clear s;
    error("Cannot create FFT CSV file.");
end

fprintf(fid, ['window_time_us,Fs_Hz,window_samples,' ...
    'P1_RMS,P1_DominantFreq_Hz,P1_PeakAmplitude,' ...
    'P1_Power_2_10Hz,P1_Power_10_30Hz,P1_Power_30_70Hz,' ...
    'P2_RMS,P2_DominantFreq_Hz,P2_PeakAmplitude,' ...
    'P2_Power_2_10Hz,P2_Power_10_30Hz,P2_Power_30_70Hz,' ...
    'TimingJitter_percent\n']);
fclose(fid);

%% Buffers
time_us = nan(maxSamples, 1);
P1 = nan(maxSamples, 1);
P2 = nan(maxSamples, 1);

n = 0;
lastRawSaved = 0;

%% Live graph
fig = figure( ...
    "Name", "Water Pipeline Sensor Monitoring", ...
    "NumberTitle", "off", ...
    "Visible", "on", ...
    "WindowState", "normal");

drawnow;

% P1 time-domain plot
ax1 = subplot(2,2,1);
h1 = animatedline(ax1, "Color", "b");
grid(ax1, "on");
xlabel(ax1, "Time (s)");
ylabel(ax1, "P1 Reading");
title(ax1, "Water Inlet Sensor - P1");

% P2 time-domain plot
ax2 = subplot(2,2,2);
h2 = animatedline(ax2, "Color", "r");
grid(ax2, "on");
xlabel(ax2, "Time (s)");
ylabel(ax2, "P2 Reading");
title(ax2, "Water Outlet Sensor - P2");

% P1 FFT plot
ax3 = subplot(2,2,3);
hFFT1 = plot(ax3, nan, nan, "b");
grid(ax3, "on");
xlabel(ax3, "Frequency (Hz)");
ylabel(ax3, "Amplitude");
title(ax3, "P1 Live FFT");
xlim(ax3, [0 100]);

% P2 FFT plot
ax4 = subplot(2,2,4);
hFFT2 = plot(ax4, nan, nan, "r");
grid(ax4, "on");
xlabel(ax4, "Frequency (Hz)");
ylabel(ax4, "Amplitude");
title(ax4, "P2 Live FFT");
xlim(ax4, [0 100]);

disp("Receiving sensor data. Close the figure to stop.");

try
    %% Continuous serial acquisition
    while ishandle(fig)

        rawLine = strtrim(readline(s));

        % Expected format: timestamp_us,P1,P2
        parts = split(rawLine, ",");

        if numel(parts) ~= 3
            continue;
        end

        values = str2double(parts);

        if any(~isfinite(values))
            continue;
        end

        %% Extract Arduino acquisition data
        t = values(1);
        p1Value = values(2);
        p2Value = values(3);

        if n >= maxSamples
            warning("Buffer full. Stopping acquisition.");
            break;
        end

        %% Store raw samples
        n = n + 1;

        time_us(n) = t;
        P1(n) = p1Value;
        P2(n) = p2Value;

        %% Update time-domain plots
        addpoints(h1, t / 1e6, p1Value);
        addpoints(h2, t / 1e6, p2Value);

        %% Periodically append raw samples to CSV
        if n - lastRawSaved >= rawSaveEvery

            fid = fopen(rawFile, 'a');

            if fid >= 0
                for k = lastRawSaved+1:n
                    fprintf(fid, '%.0f,%.0f,%.0f\n', ...
                        time_us(k), P1(k), P2(k));
                end

                fclose(fid);
                lastRawSaved = n;
            else
                warning("Unable to append raw sensor data.");
            end
        end

        %% FFT calculation
        if n >= NFFT && mod(n, updateEvery) == 0

            idx = (n-NFFT+1):n;

            % Use Arduino acquisition timestamps
            tWindow = double(time_us(idx)) * 1e-6;
            x1 = double(P1(idx));
            x2 = double(P2(idx));

            dt = diff(tWindow);

            % Validate timestamps and sensor data
            if ~all(isfinite(tWindow)) || ...
                    any(dt <= 0) || ...
                    ~all(isfinite(x1)) || ...
                    ~all(isfinite(x2))
                continue;
            end

            %% Estimate actual sampling frequency
            medianDt = median(dt);
            FsActual = 1 / medianDt;

            % Reject windows containing large timestamp gaps
            if any(dt > 1.5 * medianDt)
                title(ax3, "P1 FFT - Timestamp gap");
                title(ax4, "P2 FFT - Timestamp gap");
                continue;
            end

            timingJitter = std(dt) / medianDt * 100;

            %% Resample to a uniform time grid
            tu = (tWindow(1):medianDt:tWindow(end))';

            if numel(tu) < 16
                continue;
            end

            x1u = interp1(tWindow, x1, tu, "linear");
            x2u = interp1(tWindow, x2, tu, "linear");

            if any(~isfinite(x1u)) || any(~isfinite(x2u))
                continue;
            end

            %% Remove DC offset
            x1u = x1u - mean(x1u);
            x2u = x2u - mean(x2u);

            %% Calculate RMS
            rms1 = sqrt(mean(x1u.^2));
            rms2 = sqrt(mean(x2u.^2));

            %% Calculate FFT using Hann window
            L = length(tu);
            w = hann(L);

            Y1 = fft(x1u .* w);
            Y2 = fft(x2u .* w);

            half = floor(L/2) + 1;

            A1 = abs(Y1(1:half)) / sum(w);
            A2 = abs(Y2(1:half)) / sum(w);

            % Convert to one-sided amplitude spectrum
            if mod(L,2) == 0
                A1(2:end-1) = 2*A1(2:end-1);
                A2(2:end-1) = 2*A2(2:end-1);
            else
                A1(2:end) = 2*A1(2:end);
                A2(2:end) = 2*A2(2:end);
            end

            f = (0:half-1)' * FsActual / L;
            nyquist = FsActual / 2;

            %% Update live FFT plots
            set(hFFT1, "XData", f, "YData", A1);
            set(hFFT2, "XData", f, "YData", A2);

            xlim(ax3, [0 nyquist]);
            xlim(ax4, [0 nyquist]);

            title(ax3, sprintf( ...
                "P1 Live FFT (Fs = %.2f Hz)", FsActual));

            title(ax4, sprintf( ...
                "P2 Live FFT (Fs = %.2f Hz)", FsActual));

            %% Dominant frequency and peak amplitude
            % Ignore the DC bin
            validFreq = f > 0;

            fValid = f(validFreq);
            A1Valid = A1(validFreq);
            A2Valid = A2(validFreq);

            [peakAmp1, peakIdx1] = max(A1Valid);
            [peakAmp2, peakIdx2] = max(A2Valid);

            dominantFreq1 = fValid(peakIdx1);
            dominantFreq2 = fValid(peakIdx2);

            %% Frequency-band power
            % Integrate squared amplitude spectrum in each band.
            bands = [2 10; 10 30; 30 70];

            power1 = nan(1,3);
            power2 = nan(1,3);

            for j = 1:3

                lowHz = bands(j,1);
                highHz = min(bands(j,2), nyquist);

                if highHz <= lowHz
                    continue;
                end

                % Include band boundaries by interpolation
                inside = f > lowHz & f < highHz;
                fb = [lowHz; f(inside); highHz];

                if numel(fb) < 2
                    continue;
                end

                a1b = interp1(f, A1, fb, "linear");
                a2b = interp1(f, A2, fb, "linear");

                power1(j) = trapz(fb, a1b.^2);
                power2(j) = trapz(fb, a2b.^2);
            end

            %% Append FFT properties to CSV immediately
            fid = fopen(fftFile, 'a');

            if fid >= 0

                fprintf(fid, ...
                    ['%.0f,%.6f,%d,' ...
                     '%.8g,%.8g,%.8g,' ...
                     '%.8g,%.8g,%.8g,' ...
                     '%.8g,%.8g,%.8g,' ...
                     '%.8g,%.8g,%.8g,%.6f\n'], ...
                    time_us(n), FsActual, L, ...
                    rms1, dominantFreq1, peakAmp1, ...
                    power1(1), power1(2), power1(3), ...
                    rms2, dominantFreq2, peakAmp2, ...
                    power2(1), power2(2), power2(3), ...
                    timingJitter);

                fclose(fid);
            else
                warning("Unable to append FFT properties.");
            end

            %% Display FFT summary
            fprintf(1, ...
                ['Samples: %d | Fs: %.3f Hz | ' ...
                 'P1 peak: %.2f Hz | P2 peak: %.2f Hz\n'], ...
                n, FsActual, dominantFreq1, dominantFreq2);
        end

        %% Refresh display
        if mod(n, 10) == 0
            drawnow limitrate;
        end
    end

catch ME
    fprintf(2, "\n--- MATLAB ERROR DETAILS ---\n");
    fprintf(2, "%s\n", ...
        getReport(ME, "extended", "hyperlinks", "off"));
end

%% Save any raw samples not yet written
if n > lastRawSaved
    fid = fopen(rawFile, 'a');

    if fid >= 0
        for k = lastRawSaved+1:n
            fprintf(fid, '%.0f,%.0f,%.0f\n', ...
                time_us(k), P1(k), P2(k));
        end
        fclose(fid);
    end
end

%% Release serial port
clear s;

disp("Acquisition stopped.");
disp("Raw samples saved to pipeline_sensor_data.csv");
disp("FFT properties saved to pipeline_fft_properties.csv");
