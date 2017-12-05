xs = [-0.1; -0.05; 0; 0];
setpoint = 0.05; %desired position on bar

p = ball_beam_plant(setpoint);
c = p.balanceLQR();

v = ball_beam_visualizer(p);
%[A, B, C, D, xdot0] = p.linearize(0, p.x0, p.u0);

sys = feedback(p, c);

time_span = [0, 3];



tic;
xtraj = simulate(sys, time_span, xs);
%xtraj = simulate(p, time_span);
toc;

%output plots

Nt = 201;

t = linspace(time_span(1), time_span(2), Nt);  
xtraj_num = xtraj.eval(t);

% figure(50)
% subplot(5, 1, 1)
% hold on
% plot(t, 100*xtraj_num(1, :))
% plot(time_span, [0, 0] + p.x0(1)*100, '--k')
% hold off
% title('$$r$$', 'interpreter', 'latex')
% legend('r')
% ylabel('r (cm)')
% 
% subplot(5, 1, 2)
% hold on
% plot(t, 100*xtraj_num(2, :))
% plot(time_span, [0, 0]+p.u0, '--k')
% hold off
% title('$$\dot{r}$$', 'interpreter', 'latex')
% legend('dr')
% ylabel('rdot (cm/s)')
% 
% subplot(5, 1, 3)
% hold on
% plot(t, xtraj_num(3, :))  
% plot(time_span, [0, 0], '--k')
% hold off
% title('$$\theta$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('th (rad)')
% %ylabel('force (N)')
% legend('th')
% 
% subplot(5, 1, 4)
% hold on
% plot(t, xtraj_num(4, :))  
% plot(time_span, [0, 0], '--k')
% hold off
% title('$$\dot{\theta}$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('thdot (rad/s)')
% %ylabel('force (N)')
% legend('thdot')
% 
% 
% subplot(5, 1, 5)
% utraj_num = c.D*(xtraj_num-p.x0);
% hold on
% plot(t, utraj_num)
% plot(time_span, [0, 0] + p.u0, '--k')
% hold off
% title('$$u$$', 'interpreter', 'latex')  
% xlabel('time (s)')
% ylabel('torque (N m)')
% legend('u')


v.playback(xtraj);