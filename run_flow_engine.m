mass_flow = 1;

if mass_flow
    p = FlowEnginePlant;
else
    p = FlowEnginePlantPressure;
end

v = FlowEngineVisualizer(p);

time_span = [0, 2];
%time_span = [0, 1];
%time_span = [0, 5];

n = 5;

%find region of attraction
ROA = 0;

[c, V] = hoverLQR(p, ROA);
sys = feedback(p, c);
tic;
xtraj = simulate(sys, time_span);
toc;
%output plots
figure(35)

Nt = 201;

t = linspace(time_span(1), time_span(2), Nt);  
xtraj_num = xtraj.eval(t);


subplot(5, 1, 1)
hold on
plot(t, 100*xtraj_num(1, :))
plot(time_span, [0, 0] + p.x0(1)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
legend('z')
ylabel('heave (cm)')

subplot(5, 1, 2)
hold on
plot(t, 100*xtraj_num(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
legend('dz')
ylabel('vertical speed (cm/s)')


subplot(5, 1, 3)
hold on
plot(t, xtraj_num(3, :)/1000)  
plot(time_span, [0, 0] + p.x0(3)/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time')
ylabel('pressure (kPa)')
%ylabel('force (N)')
legend('p')

subplot(5, 1, 4)
utraj_num = c.D*xtraj_num;
utraj_shift = utraj_num + p.u0;
hold on
plot(t, utraj_shift)  
plot(time_span, [0, 0] + p.u0, '--k')
hold off
title('$$\dot{m_{in}} (u)$$', 'interpreter', 'latex')  
xlabel('time')
ylabel('mass flow rate (kg/s)')
legend('u')

subplot(5, 1, 5)
xtraj_0 = xtraj_num - p.x0;
Vtime = dot(xtraj_0, V.S*xtraj_0);
hold on
plot(t, Vtime);
plot(time_span, [0, 0], '--k')
hold off
title('$$V(x)$$', 'interpreter', 'latex')
xlabel('time')
ylabel('Lyapunov Energy')
legend('V(x)')

v.playback(xtraj);