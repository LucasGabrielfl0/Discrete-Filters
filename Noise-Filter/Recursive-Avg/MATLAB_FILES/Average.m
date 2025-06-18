%% Creating Dataset with Gaussian Noise
time_ms   = 0:1:1000;
SineFunc  = (3.3/2)*( sin(0.02*time_ms) + 1 );
NoisyData = zeros(size(time_ms));

for c=1:1:1001
    NoisyData(c) = SineFun(c) + 0.06*randn;
end

% Plot
hold on
grid on
plot(time_ms, SineFun);
plot(time_ms, NoisyData);

%% Batch
k = 0; % Number Of i
alpha = (k-1)/k;
alpha = (k-1)/k;

Xk_avg = alpha * Xk_1 + (1 - alpha) * Xk;

%% Cumulative Average (Recursive)
RecAvg = zeros(size(time_ms));

Avg_Data= 0.5*ones(size(time_ms));

% Plot
hold on
grid on
plot(time_ms, NoisyData,"r:*");
plot(time_ms, Avg_Data);

legend("Original Signal", "Cumulative Average")
xlabel("Time")
ylabel("Output")
title("Average")

%% Simple Moving Average
% Batch Average of the last N values

MovAvg = zeros(size(time_ms));
num= 10;
Xkn = zeros(1, num);
count = 0;

MovAvg(1) = 0;
for k=2:1:1001
    MovAvg(k) = MovAvg(k - 1) + ( NoisyData(k) - Xkn )/num;
    count= count+1;

    Xkn[num] =

    if (count > num )
        Xkn = NoisyData(k-10);
        count = 0;
    end
end

% Plot
hold on
grid on
plot(time_ms, NoisyData);
plot(time_ms, MovAvg);


legend("Original Signal", "Moving Average")
xlabel("Time")
ylabel("Output")
title("Moving Average")

%% Exponential Moving Average
% Batch Average of the last N values, but gives more weight to recent
% values


MovAvg = zeros(size(time_ms));


%% Low Pass Filter




%% Compare All
close all
hold on
grid on

plot(time_ms, NoisyData);       % Cumulative Average

plot(time_ms, RecAvg);          % Cumulative Average
plot(time_ms, MovAvg);          % Simple Moving Average
plot(time_ms, ExpMovAvg);       % Exponential Moving Average
plot(time_ms, LowPassFilt);     % Low Pass Filter


legend("Original Signal", "Filtered Signal [Average]")
xlabel("Time")
ylabel("Output")
title("Batch Average")
