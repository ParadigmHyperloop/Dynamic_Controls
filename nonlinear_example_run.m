p = nonlinear_example;

time_span = [0, 4];

[c, V] = hoverLQR(p);
sys = feedback(p, c);

xtraj = simulate(sys, time_span);

figure(30)
clf;
Nt = 201;
t = linspace(time_span(1), time_span(2), Nt);  
xtraj_num = xtraj.eval(t);

subplot(3, 1, 1)
hold on
plot(t, xtraj_num(1, :))
plot(time_span, [0, 0] + p.x0(1), '--k')
hold off
title('$$z$$', 'interpreter', 'latex')
legend('z')
ylabel('heave (cm)')

subplot(3, 1, 2)
utraj_num = c.D*xtraj_num;
utraj_shift = utraj_num + p.u0;
hold on
plot(t, utraj_shift)  
plot(time_span, [0, 0] + p.u0, '--k')
hold off
title('$$u$$', 'interpreter', 'latex')  
xlabel('time')
ylabel('pressure (N/cm^2)')
%ylabel('force (N)')
legend('p')

subplot(3, 1, 3)
xtraj_0 = xtraj_num - p.x0;
%Vtime = dot(xtraj_0, V.S*xtraj_0);
Vtime = V.S * (xtraj_0).^2;
hold on
plot(t, Vtime);
plot(time_span, [0, 0], '--k')
hold off
title('$$V(x)$$', 'interpreter', 'latex')
xlabel('time')
ylabel('Lyapunov Energy')
legend('V(x)')