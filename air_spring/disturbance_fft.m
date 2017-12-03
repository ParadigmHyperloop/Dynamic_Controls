%% Vehicle and Track Parameters
panel_length = 12.5; %12.5'
panel_length_gap = 0.25/12; %0.25"
panel_gap_depth = 0.5/12; %0.5"


total_length = panel_length + panel_length_gap;
duty_cycle = 100*panel_length/(total_length);

%vehicle_speed = 60; %60 mph
vehicle_speed = 400; %60 mph


fps_over_mph = 0.6818; % 1ft/s = 0.6818 mph
v = vehicle_speed * fps_over_mph;

%% Sampling
tmax = 1;
N = 100000;
t = linspace(0, tmax, N);
%w = zeros(size(t));


%% Disturbances
x = v.*t;
%x = zeros(size(t));
ws = square(2*pi.*x/total_length, duty_cycle);

w = (ws/2 - 0.5)*panel_gap_depth;

%% Fourier
Fs = N/tmax;
y = fft(x);
y = fftshift(y);
yo = y(N/2:end);

f = Fs*(0:(N/2))/N;

f_ind = 100;

%% Plotting Output
figure(1)
%plot(x, w)
subplot(2, 1, 1)
plot(t, w)

subplot(2, 1, 2)
plot(f(1:f_ind), abs(yo(1:f_ind)))