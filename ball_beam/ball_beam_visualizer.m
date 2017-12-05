classdef ball_beam_visualizer < Visualizer
% Implements the draw function for the Bar Rocket (Planar Quadrotor) model

  properties
    L=.25;    % beam length
    W = 0.01; % beam width
    r_ball = 0.02; %ball radius
    r_pin  = 0.004; %pin radius
    setpoint;
  end

  methods
    function obj = ball_beam_visualizer(plant)
      typecheck(plant,'ball_beam_plant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.L = plant.L;
      obj.r_ball = plant.r_ball;
      obj.setpoint = plant.x0(1);
    end
    
    function draw(obj,t,x)
      % Draw the quadrotor.  
      persistent hFig beam circ;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        beam = [obj.L*[1 -1 -1 1]; obj.W*[1 1 -1 -1]];
        t = linspace(0, 2*pi, 50);
        circ = [cos(t); sin(t)];
      end
            
      sfigure(hFig); cla; hold on; view(0,90);
      
      %r = [cos(x(2)), -sin(x(2)); sin(x(2)), cos(x(2))];

      r = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];

      %plot the beam
      p = r*beam;
      patch(p(1, :), p(2, :), 'k', 'FaceColor',[0 0 0]);
      
      %plot the ball
      ball_center = [x(1); obj.W + obj.r_ball];
      p = r*ball_center;
      patch(obj.r_ball*circ(1, :) + p(1), obj.r_ball*circ(2, :) + p(2), 'k', 'FaceColor', [0 0 0.5]);      
     
      %plot the pin
      patch(obj.r_pin*circ(1, :), obj.r_pin*circ(2, :), 'k', 'FaceColor', [0.7 0 0]);      
%      
      %plot the setpoint
      p = r*[obj.setpoint; 0];
      patch(obj.r_pin*circ(1, :) + p(1), obj.r_pin*circ(2, :) + p(2), 'k', 'FaceColor', [0.6 0.6 0.6]);      
%     

      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      %set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-obj.L*1.25 obj.L*1.25 -obj.L obj.L]);
      xlabel('ball position (m)')
      ylabel('height (m)')
      
      drawnow;
    end    
  end
  
end
