xs = [0.15; 0.0; 0.0; 0.0];
setpoint = -0.1; %desired position on bar

%observer control
use_observer = 1;
num_output = 2; %1: r, 2: r, th
save_output = 0;

if use_observer
    %observer stuff
    p = ball_beam_plant_observer(setpoint, num_output);
    [A, B, C, D] = p.linearize(0, p.x0, p.u0);
    A = full(A);
    B = full(B);
    
    c = p.balanceLQR();
    F = c.D;
    
    L = place(A', C', 10*eig(A+B*F));    
    
    observer = LuenbergerObserver(p, L', []);
    %observer = LuenbergerObserver(p, L, []);
    observer.forward_model = LinearSystem(A, B, [], [], C, []); %need to figure out how this works for hinf stuff    
    
    sys = mimoFeedback(mimoCascade(p, observer), c);

else
    p = ball_beam_plant(setpoint);
    [A, B, C, D] = p.linearize(0, p.x0, p.u0);
    
    c = p.balanceLQR();
    sys = feedback(p, c);
end


v = ball_beam_visualizer(p);

time_span = [0, 3];
dt = 0.025;


tic;
ytraj = simulate(sys, time_span);
%ytraj = simulate(sys, time_span, xs);
%ytraj = simulate(p, time_span);
toc;

%output plots

Nt = 201;

%t = linspace(time_span(1), time_span(2), Nt);  
%xtraj_num = ytraj.eval(t);

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

%v_options = struct();
%v_options.slider = true;
%v_options.display_dt = 0.1;
v.playback(ytraj);
%v.playbackMovie(ytraj, 'ball_beam_movie.swf')

%real inefficient solution
if save_output
    for t = time_span(1):dt:time_span(2)
        x = ytraj.eval(t);
        v.drawWrapper(t, x);
        if t == 0
            t_name = '0.000';
        else
            t_name = num2str(t, '%03f');
        end
        print(['bar_rocket_drake/ball_beam/img/ball_beam_t_', t_name, '.png'], '-dpng')
    end
end

%export to sequence of images