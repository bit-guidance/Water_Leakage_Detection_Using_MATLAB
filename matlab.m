
clc;
clear;
close all;

%% Serial configuration
port = "COM3";       % Change to your Arduino COM port
baudRate = 115200;

s = serialport(port, baudRate);
configureTerminator(s, "LF");
flush(s);

%% Buffers
maxSamples = 100000;

time_us = nan(maxSamples, 1);
P1 = nan(maxSamples, 1);
P2 = nan(maxSamples, 1);

n = 0;

%% Live graph
figure("Name", "Water Pipeline Sensor Monitoring");

subplot(2,1,1);
h1 = animatedline("Color", "b");
grid on;
xlabel("Time (s)");
ylabel("P1 Reading");
title("Water Inlet Sensor - P1");

subplot(2,1,2);
h2 = animatedline("Color", "r");
grid on;
xlabel("Time (s)");
ylabel("P2 Reading");
title("Water Outlet Sensor - P2");

disp("Receiving sensor data. Close the figure to stop.");

try
    while ishandle(gcf)

        % Read one complete serial line
        rawLine = strtrim(readline(s));

        % Expected format: timestamp_us,P1,P2
        parts = split(rawLine, ",");

        % Ignore header, incomplete or invalid messages
        if numel(parts) ~= 3
            continue;
        end

        values = str2double(parts);

        if any(~isfinite(values))
            continue;
        end

        % Convert received timestamp from microseconds
        t = values(1);
        p1Value = values(2);
        p2Value = values(3);

        % Store in arrays
        n = n + 1;

        if n > maxSamples
            warning("Buffer full. Stopping acquisition.");
            break;
        end

        time_us(n) = t;
        P1(n) = p1Value;
        P2(n) = p2Value;

        % Plot time in seconds
        addpoints(h1, t / 1e6, p1Value);
        addpoints(h2, t / 1e6, p2Value);

        % Refresh the display
        if mod(n, 10) == 0
            drawnow limitrate;
        end
    end

catch ME
    disp("Acquisition stopped.");
    disp(ME.message);
end

%% Keep only valid samples
time_us = time_us(1:n);
P1 = P1(1:n);
P2 = P2(1:n);

%% Save acquired data
data = table(time_us, P1, P2);
writetable(data, "pipeline_sensor_data.csv");

disp("Data saved to pipeline_sensor_data.csv");

%% Release the serial port
clear s;
