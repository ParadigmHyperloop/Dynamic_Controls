%back to basics

p = AirSpringPlant1d_slow;

%controller
ROA = 0;
[c,V] = p.hoverLQR(ROA);
Lam_V = eig(V.S)


time_span = [0, 5];

%dyn = @(u) 
x0 = p.getInitialState();
Nt = 200;
%t = linspace(time_span(1), time_span(2), Nt);  
%sp = spline(T, X'); 

%xtraj_num = sp.eval(t);


%% Open Loop
dyn = @(t, x) p.dynamics(t, x, p.u0);
[T, X] = ode45(dyn, time_span, x0, []);

t = T;
xtraj = X';

figure(7)
clf

subplot(3, 1, 1)
%subplot(4, 1, 1)
%subplot(5, 1, 1)
hold on
plot(t, 100*xtraj(1, :))
plot(time_span, [0, 0] + p.x0(1)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
%legend('z')
ylabel('heave (cm)')

subplot(3, 1, 2)
%subplot(4, 1, 2)
%subplot(5, 1, 2)
hold on
plot(t, 100*xtraj(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
%legend('dz')
ylabel('vertical speed (cm/s)')

subplot(3, 1, 3)
%subplot(4, 1, 3)
%subplot(5, 1, 3)
hold on
plot(t, xtraj(3, :)/p.pressure_scale/1000)  
plot(time_span, [0, 0] + p.x0(3)/p.pressure_scale/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('bagpressure (kPa)')
%ylabel('force (N)')
%legend('p')


%% Closed Loop
dyn_cl = @(t, x) p.dynamics(t, x, p.u0 + c.D*(x - p.x0));

[T_cl, X_cl] = ode45(dyn_cl, time_span, x0, []);

t = T_cl;
xtraj = X_cl';

figure(8)
clf

%subplot(3, 1, 1)
%subplot(4, 1, 1)
subplot(5, 1, 1)
hold on
plot(t, 100*xtraj(1, :))
plot(time_span, [0, 0] + p.x0(1)*100, '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
%legend('z')
ylabel('heave (cm)')

%subplot(3, 1, 2)
%subplot(4, 1, 2)
subplot(5, 1, 2)
hold on
plot(t, 100*xtraj(2, :))
plot(time_span, [0, 0], '--k')
hold off
title('$$\dot{z}$$', 'interpreter', 'latex')
%legend('dz')
ylabel('vertical speed (cm/s)')

%subplot(3, 1, 3)
%subplot(4, 1, 3)
subplot(5, 1, 3)
hold on
plot(t, xtraj(3, :)/p.pressure_scale/1000)  
plot(time_span, [0, 0] + p.x0(3)/p.pressure_scale/1000, '--k')
hold off
title('$$p$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('bagpressure (kPa)')
%ylabel('force (N)')
%legend('p')

%subplot(4, 1, 4)
subplot(5, 1, 4)
hold on
plot(t, xtraj(4, :))  
plot(time_span, [0, 0] + p.x0(4), '--k')
hold off
title('$$\dot{m}_{in}^{actual}$$', 'interpreter', 'latex')  
xlabel('time (s)')
ylabel('mass flow (kg/s)')
%ylabel('force (N)')
%legend('p')

subplot(5, 1, 5)
utraj_num = c.D*(xtraj-p.x0) + p.u0;
hold on
plot(t, utraj_num)
plot(time_span, [0, 0] + p.u0, '--k')
hold off
title('$$\dot{m}_{in}^{desired}$$', 'interpreter', 'latex')   
xlabel('time (s)')
ylabel('mass flow (kg/s)')