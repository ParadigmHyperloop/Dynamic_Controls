%back to basics

%plant
%p = AirSpringPlant1d_slow;
p0 = AirSpringPlant1d_slow;
p = AirSpringPlant1d_disturbed_2;

%controller
ROA = 0;

[c,V] = p.hoverLQR(ROA);
Lam_V = eig(V.S);

%disturbance
seed = randi(1e9);
%seed = 10;

%v = 5;
%v = 80;
%v = 26.8224;   %60 mph
%v = 180;       %400 mph
%v = 268;       %600 mph
v = 312.928;    %700 mph
d = disturbance_input_3(p, v, seed);


%setup
time_span = [0, 5];
%time_span = [0, 20];

x0 = p.getInitialState();
Nt = 200;

%disturbance stuff
%this is truly awful
rng(seed)
x_max = v*time_span(2);
num_steps = floor(x_max/d.track_period);
steps = [0 (rand(1, num_steps)-0.5) * 2 * d.track_drift_incremental];
track_drift = cumsum(steps);
track_drift_breaks = x_max .* (0:num_steps)/num_steps/v;

 
 
%% Open Loop
dyn0 = @(t, x) p.dynamics(t, x, p0.u0(1));
dyn = @(t, x) p.dynamics(t, x, [p.u0(1); d.output(t)]);

%[T0, X0] = ode45(dyn, time_span, x0, []);
[T, X] = ode45(dyn, time_span, x0, []);

t = T;
xtraj = X';

figure(7)
clf

subplot(3, 1, 1)
hold on
plot(t, 100*xtraj(1, :))
%plot(time_span, [0, 0] + p.x0(1)*100, '--k')
stairs(track_drift_breaks, (p.x0(1) + track_drift)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
ylabel('heave (cm)')

subplot(3, 1, 2)
hold on
plot(t, 100*xtraj(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
ylabel('vertical speed (cm/s)')

subplot(3, 1, 3)
hold on
plot(t, xtraj(3, :)/p.pressure_scale/1000)  
plot(time_span, [0, 0] + p.x0(3)/p.pressure_scale/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('bagpressure (kPa)')

% %% Closed Loop
% dyn_cl = @(t, x) p.dynamics(t, x, disturbed_controlled_output(x, c.D, d.output(t), p.x0, p.u0));
% 
% [T_cl, X_cl] = ode45(dyn_cl, time_span, x0, []);
% 
% t = T_cl;
% xtraj = X_cl';
% 
% figure(8)
% clf
% 
% %subplot(3, 1, 1)
% %subplot(4, 1, 1)
% subplot(5, 1, 1)
% hold on
% plot(t, 100*xtraj(1, :))
% plot(time_span, [0, 0] + p.x0(1)*100, '--k')
% hold off
% title('$$z$$', 'interpreter', 'latex')
% %legend('z')
% ylabel('heave (cm)')
% 
% %subplot(3, 1, 2)
% %subplot(4, 1, 2)
% subplot(5, 1, 2)
% hold on
% plot(t, 100*xtraj(2, :))
% plot(time_span, [0, 0], '--k')
% hold off
% title('$$\dot{z}$$', 'interpreter', 'latex')
% %legend('dz')
% ylabel('vertical speed (cm/s)')
% 
% %subplot(3, 1, 3)
% %subplot(4, 1, 3)
% subplot(5, 1, 3)
% hold on
% plot(t, xtraj(3, :)/p.pressure_scale/1000)  
% plot(time_span, [0, 0] + p.x0(3)/p.pressure_scale/1000, '--k')
% hold off
% title('$$p$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('bagpressure (kPa)')
% %ylabel('force (N)')
% %legend('p')
% 
% %subplot(4, 1, 4)
% subplot(5, 1, 4)
% hold on
% plot(t, xtraj(4, :))  
% plot(time_span, [0, 0] + p.x0(4), '--k')
% hold off
% title('$$\dot{m}_{in}^{actual}$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('mass flow (kg/s)')
% %ylabel('force (N)')
% %legend('p')
% 
% subplot(5, 1, 5)
% utraj_num = zeros(size(t));
% for i = 1:length(t)
%     u_curr = disturbed_controlled_output(xtraj(:, i), c.D, d.output(t(i)), p.x0, p.u0);
%     utraj_num(i) = u_curr(1);
% end
% hold on
% plot(t, utraj_num)
% plot(time_span, [0, 0] + p.u0, '--k')
% hold off
% title('$$\dot{m}_{in}^{desired}$$', 'interpreter', 'latex')   
% xlabel('time (s)')
% ylabel('mass flow (kg/s)')
% 
