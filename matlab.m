
clc;
clear;
close all;

%% Serial configuration
port = "COM19";       % Change to your Arduino COM port
baudRate = 115200;

s = serialport(port, baudRate);
configureTerminator(s, "LF");
flush(s);

%% Configuration
maxSamples = 100000;

NFFT = 1024;              % Samples per FFT window
updateEvery = 100;        % Refresh FFT every 100 new samples

%% Buffers
time_us = nan(maxSamples, 1);
P1 = nan(maxSamples, 1);
P2 = nan(maxSamples, 1);

n = 0;

%% Live graph
fig = figure("Name", "Water Pipeline Sensor Monitoring");

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
    while ishandle(fig)

        %% Read one complete serial line
        rawLine = strtrim(readline(s));

        % Expected format: timestamp_us,P1,P2
        parts = split(rawLine, ",");

        % Ignore headers, incomplete or invalid messages
        if numel(parts) ~= 3
            continue;
        end

        values = str2double(parts);

        if any(~isfinite(values))
            continue;
        end

        %% Extract Arduino acquisition data
        t = values(1);       % Acquisition timestamp in microseconds
        p1Value = values(2);
        p2Value = values(3);

        %% Store data
        n = n + 1;

        if n > maxSamples
            warning("Buffer full. Stopping acquisition.");
            n = n - 1;
            break;
        end

        time_us(n) = t;
        P1(n) = p1Value;
        P2(n) = p2Value;

        %% Update time-domain plots
        addpoints(h1, t / 1e6, p1Value);
        addpoints(h2, t / 1e6, p2Value);

        %% Update FFT
        if n >= NFFT && mod(n, updateEvery) == 0

            idx = (n-NFFT+1):n;

            % Use Arduino acquisition timestamps
            tWindow = double(time_us(idx)) * 1e-6;
            x1 = double(P1(idx));
            x2 = double(P2(idx));

            dt = diff(tWindow);

            % Validate timestamp order and signal data
            if all(isfinite(tWindow)) && ...
                    all(dt > 0) && ...
                    all(isfinite(x1)) && ...
                    all(isfinite(x2))

                %% Estimate actual sampling rate
                medianDt = median(dt);
                FsActual = 1 / medianDt;

                timingJitter = std(dt);
                relativeJitter = timingJitter / medianDt;

                %% Check for unusually large sampling gaps
                % Reject windows with a gap exceeding 1.5 times
                % the median interval. Adjust if needed after testing.
                if any(dt > 1.5 * medianDt)
                    title(ax3, "P1 FFT - Invalid timestamp gap");
                    title(ax4, "P2 FFT - Invalid timestamp gap");

                else
                    %% Resample using acquisition timestamps
                    % Use uniform spacing based on measured timestamps
                    tu = (tWindow(1):medianDt:tWindow(end))';

                    if length(tu) >= 16

                        x1u = interp1(tWindow, x1, tu, "linear");
                        x2u = interp1(tWindow, x2, tu, "linear");

                        %% FFT processing
                        L = length(tu);

                        % Remove DC offset
                        x1u = x1u - mean(x1u);
                        x2u = x2u - mean(x2u);

                        % Hann window
                        w = hann(L);

                        Y1 = fft(x1u .* w);
                        Y2 = fft(x2u .* w);

                        %% One-sided amplitude spectra
                        half = floor(L/2) + 1;

                        A1 = abs(Y1(1:half)) / sum(w);
                        A2 = abs(Y2(1:half)) / sum(w);

                        if mod(L,2) == 0
                            A1(2:end-1) = 2*A1(2:end-1);
                            A2(2:end-1) = 2*A2(2:end-1);
                        else
                            A1(2:end) = 2*A1(2:end);
                            A2(2:end) = 2*A2(2:end);
                        end

                        %% Frequency axis
                        f = (0:half-1)' * FsActual / L;

                        %% Update FFT plots
                        set(hFFT1, "XData", f, "YData", A1);
                        set(hFFT2, "XData", f, "YData", A2);

                        xlim(ax3, [0 FsActual/2]);
                        xlim(ax4, [0 FsActual/2]);

                        title(ax3, sprintf( ...
                            "P1 Live FFT (Fs = %.2f Hz)", FsActual));

                        title(ax4, sprintf( ...
                            "P2 Live FFT (Fs = %.2f Hz)", FsActual));

                        %% Report timing information                      
                        fprintf(1, 'Samples: %d | Fs: %.3f Hz | Jitter: %.3f%% | FFT length: %d\n', ...
                            n, FsActual, 100*relativeJitter, L);

                    end
                end
            end
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

    for k = 1:numel(ME.stack)
        fprintf(2, "File: %s | Line: %d | Function: %s\n", ...
            ME.stack(k).file, ...
            ME.stack(k).line, ...
            ME.stack(k).name);
    end
end


%% Keep only valid samples
time_us = time_us(1:n);
P1 = P1(1:n);
P2 = P2(1:n);

%% Save acquired data
data = table(time_us, P1, P2);
writetable(data, "pipeline_sensor_data.csv");

disp("Data saved to pipeline_sensor_data.csv");

%% Release serial port
clear s;
