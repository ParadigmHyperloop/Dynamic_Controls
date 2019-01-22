p = BallBeamSimple(0.0);
xs = [0.1; 0.1];

[c] = p.balanceLQRD();
c.setSampleTime([p.Ts; 0]);

sys = feedback(p, c);

time_span = [0, 3];
ytraj = simulate(sys, time_span, xs);


Nt = 201;
t = linspace(time_span(1), time_span(2), Nt);
xtraj_num = ytraj.eval(t);

figure(51)
subplot(2, 1, 1)
hold on
plot(t, xtraj_num(1, :))
plot(time_span, [0, 0] + p.x0(1), '--k')
hold off
title('$$r$$', 'interpreter', 'latex')
legend('r')
ylabel('r (cm)')

subplot(2, 1, 2)
hold on
plot(t, xtraj_num(2, :))
plot(time_span, [0, 0]+p.u0, '--k')
hold off
title('$$\dot{r}$$', 'interpreter', 'latex')
legend('dr')
ylabel('rdot (cm/s)')