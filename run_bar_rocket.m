%function c = run_bar_rocket
%adapted from Drake Examples

p = BarRocketPlant;
v = BarRocketVisualizer(p);

time_span = [0, 4];

n=1;

%if checkDependency('sedumi')
%if 1
if 0
  [c,V] = hoverLQR(p);
  sys = feedback(p,c);
  
  figure(1); plotFunnel(V,struct('plotdims',[3 6]));
  xlabel('$\theta$','interpreter','latex');
  ylabel('$\dot\theta$','interpreter','latex');
  %xlabel('$$ \theta $$','interpreter','latex');
  %xlabel('$$ \dot{\theta} $$','interpreter','latex');
  
  
  x0=zeros(6,1);
  y=getLevelSet(V,[],struct('num_samples',n));
  for i=1:n
    xtraj=simulate(sys,time_span,x0 + .99*y(:,i));
    figure(1); fnplt(xtraj,[3 6]);
    figure(25);
    v.playback(xtraj);
    
    ts = xtraj.getBreaks();Vs=zeros(1,length(ts));
    for i=1:length(ts)
      Vs(i) = V.eval(ts(i),xtraj.eval(ts(i)));
    end
    if any(diff(Vs)>1e-4)
      diff(Vs)
      error('V(t,x) increased at some point in the trajectory from an initial condition in the verified ROA.');
    end
  end
  
else
  c = hoverLQR(p);
  sys = feedback(p,c);
  
  for i=1:n
    xtraj = simulate(sys,time_span);
    v.playback(xtraj);
    
  end
  
  %output plots
  figure(35)
  t = linspace(time_span(1), time_span(2), 201);  
  xtraj_num = xtraj.eval(t);
  
  subplot(4, 1, 1)
  hold on
  plot(t, xtraj_num(1:2, :))
  plot(time_span, [0, 0], '--k')
  hold off
  title('x and dx')
  legend('x', 'dx')
  ylabel('sway (m)')
  
  subplot(4, 1, 2)
  hold on
  plot(t, xtraj_num(3:4, :))
  plot(time_span, [0, 0], '--k')
  hold off
  title('z and dz')
  legend('z', 'dz')
  ylabel('heave (m)')
  
  subplot(4, 1, 3)
  hold on
  plot(t, xtraj_num(5:6, :))  
  plot(time_span, [0, 0], '--k')
  hold off
  title('theta and dtheta')
  legend('theta', 'dtheta')
  ylabel('angle (rad)')
  
  subplot(4, 1, 4)
  utraj_num = c.D*xtraj_num;
  hold on
  plot(t, utraj_num)  
  plot(time_span, [0, 0], '--k')
  hold off
  title('u (force input)')  
  xlabel('time (x)')
  ylabel('force (N)')
  legend('u left', 'u right')
end


