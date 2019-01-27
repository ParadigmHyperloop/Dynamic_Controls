active_control = 0;

%p = AirSpringPlant1d;
p = AirSpringPlant1d_slow;

% v = FlowEngineVisualizer(p);

time_span = [0, 1];
%time_span = [0, 1.4290667];
%time_span = [0, 1.78];
%time_span = [0, 10];

%time_span = [0, 5];

n = 1;

%find region of attraction
ROA = 0;
if active_control
    [c, V] = hoverLQR(p, ROA);
    sys = feedback(p, c);
else
    sys = p;
end

tic;
%xtraj = simulate(sys, time_span);
%xtraj = simulate(p, time_span);
x0 = p.getInitialState();
dyn = @(t, x) p.dynamics(t, x, p.u0);
[T, X] = ode45(dyn, time_span, x0, []);

toc;
%output plots
figure(35)
clf
Nt = 201;

t = T;
xtraj_num = X';
%t = linspace(time_span(1), time_span(2), Nt);  
%xtraj_num = xtraj.eval(t);


subplot(3, 1, 1)
%subplot(4, 1, 1)
hold on
plot(t, 100*xtraj_num(1, :))
plot(time_span, [0, 0] + p.x0(1)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
%legend('z')
ylabel('heave (cm)')

subplot(3, 1, 2)
%subplot(4, 1, 2)
hold on
plot(t, 100*xtraj_num(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
%legend('dz')
ylabel('vertical speed (cm/s)')

subplot(3, 1, 3)
hold on
plot(t, xtraj_num(3, :)/p.pressure_scale/1000)  
plot(time_span, [0, 0] + p.x0(3)/p.pressure_scale/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('bagpressure (kPa)')
%ylabel('force (N)')
%legend('p')



% v.playback(xtraj);