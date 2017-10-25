classdef SpringRocketVisualizer < Visualizer
% Implements the draw function for the Bar Rocket (Planar Quadrotor) model

  properties
    L;  % moment arm
    h=.025;
  end

  methods
    function obj = SpringRocketVisualizer(plant)
      typecheck(plant,'SpringRocketPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.L = plant.L;
    end
    
    function draw(obj,t,x)
      % Draw the quadrotor.  
      %persistent hFig base pin prop;
      persistent hFig base skate;
      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        base = [1.2*obj.L*[1 -1 -1 1]; obj.h*[1 1 -1 -1]];
        skate = [0.2*obj.L*[1 -0.6 -0.6 1]; obj.h/2*[1 1 -1 -1]];
        
        

      end
            
      sfigure(hFig); cla; hold on; view(0,90);
      
      r = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];
      
      %spring?

      p = r*base;
      patch(x(1)+p(1,:), x(2)+p(2,:),1+0*p(1,:),'b','FaceColor',[0 0 0])
      
%       p = r*[obj.L+pin(1,:);pin(2,:)];
%       patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
%       p = r*[-obj.L+pin(1,:);pin(2,:)];
%       patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
%       
%       p = r*[obj.L+prop(1,:);prop(2,:)];
%       patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
%       p = r*[-obj.L+prop(1,:);prop(2,:)];
%       patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
       
      %skates
      %left skate
      p = r*[-obj.L/2 + x(4) + skate(1, :), x(5) + skate(2, :)];
      patch(x(1) + p(1, :), x(2) + p(2, :), [0.8, 0, 0], 'EdgeColor','none');
       
      %right skate
      p = r*[obj.L/2 + x(6) + skate(1, :), x(7) + skate(2, :)];
      patch(x(1) + p(1, :), x(2) + p(2, :), [0.8, 0, 0], 'EdgeColor','none');
       
      %springs
%       spring_width = 0.03;
%       spring_x = [0; 0; x(1); x(1)];
%       spring_z = [spring_width; -spring_width; -spring_width; spring_width] + x(2);
%       patch(spring_x, spring_z, [0.4, 0.4, 0.4]);
%       end
      
      %left spring
      spring1 = [-obj.L/2, x(4) - x(1); 0, x(5) - x(2)];
      p = r*spring1;
      line([x(1) + p(1, :), x(2) + p(2, :)],...
                    'LineWidth', 3,...
                    'Color', [.7 .7 .7]);
                    
      %right spring
      spring2 = [-obj.L/2, x(6) - x(1); 0, x(7) - x(2)];
      p = r*spring2;                    
      line([x(1) + p(1, :), x(2) + p(2, :)],...
                    'LineWidth', 3,...
                    'Color', [.7 .7 .7]);
      
      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-2.0 2.0 -1.0 1.0]);
      drawnow;
    end    
  end
  
end
