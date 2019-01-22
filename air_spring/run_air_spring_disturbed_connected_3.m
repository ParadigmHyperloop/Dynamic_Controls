time_span = [0, 5];

seed = randi(1e9);
%seed = 90643942;
%seed = 10;

%p = AirSpringPlant1d_disturbed;
%c = disturbance_input(p);

%v = 26.8224; %60 mph
%v = 26.8;
%v = 5;
%v = 80;
%v = 180; %400 mph
%v = 268; % 600 mph
v = 312.928; %700 mph
p = AirSpringPlant1d_disturbed_2;
w = disturbance_input_2(p, v, seed);
%c = pod_controller(p);


%connections
connection = [struct() struct()];
connection(1).from_output = 1;
connection(1).to_input = 2;
connection(2).from_output = 2;
connection(2).to_input = 3;
sys = mimoCascade(w, p, connection);


%w = w.setOutputFrame(p.getInputFrame);
%p = p.setOutputFrame(w.getInputFrame);

%disturbance stuff
%this is truly awful
rng(seed)
x_max = v*time_span(2);
num_steps = floor(x_max/w.track_period);
steps = [0 (rand(1, num_steps)-0.5) * 2 * w.track_drift_incremental];
track_drift = cumsum(steps);
track_drift_breaks = x_max .* (0:num_steps)/num_steps/v;



%vis = FlowEngineVisualizer(p);

n = 1;

%[c, V] = hoverLQR(p, ROA);
tic;
xtraj = simulate(sys, time_span);
%xtraj = simulate(p, time_span);
toc;
%output plots
figure(35)

Nt = 201;

t = linspace(time_span(1), time_span(2), Nt);  
xtraj_num = xtraj.eval(t);


%subplot(5, 1, 1)
%subplot(4, 1, 1)
subplot(3, 1, 1)
hold on
plot(t, 100*xtraj_num(1, :))
plot(time_span, [0, 0] + p.x0(1)*100, ':k')
stairs(track_drift_breaks, (p.x0(1) + track_drift)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
legend('z')
ylabel('heave (cm)')

%subplot(5, 1, 2)
%subplot(4, 1, 2)
subplot(3, 1, 2)
hold on
plot(t, 100*xtraj_num(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
legend('dz')
ylabel('vertical speed (cm/s)')

%subplot(4, 1, 3)
subplot(3, 1, 3)
hold on
plot(t, xtraj_num(3, :)/1000)  
plot(time_span, [0, 0] + p.x0(3)/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('bagpressure (kPa)')
%ylabel('force (N)')
legend('p')

% subplot(4, 1, 4)
% hold on
% plot(t, xtraj_num(4, :))  
% plot(time_span, [0, 0] + p.x0(4), '--k')
% hold off
% title('$$\dot{m}_{in}$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('massflowin (kg/s)')
% %ylabel('force (N)')
% legend('min')

% vis.playback(xtraj);