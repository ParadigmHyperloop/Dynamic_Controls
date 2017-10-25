classdef BarRocketVisualizer < Visualizer
% Implements the draw function for the Bar Rocket (Planar Quadrotor) model

  properties
    L=.25;  % moment arm
    kx = 0;
  end

  methods
    function obj = BarRocketVisualizer(plant)
      typecheck(plant,'BarRocketPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.L = plant.L;
      obj.kx = plant.kx;
    end
    
    function draw(obj,t,x)
      % Draw the quadrotor.  
      persistent hFig base pin prop;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        base = [1.2*obj.L*[1 -1 -1 1]; .025*[1 1 -1 -1]];
        pin = [.005*[1 1 -1 -1]; .1*[1 0 0 1]];
        a = linspace(0,2*pi,50);
        prop = [obj.L/1.5*cos(a);.1+.02*sin(2*a)];
        

      end
            
      sfigure(hFig); cla; hold on; view(0,90);
      
      r = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];
      
      %spring?
      if obj.kx > 0
          spring_width = 0.03;
          spring_x = [0; 0; x(1); x(1)];
          spring_z = [spring_width; -spring_width; -spring_width; spring_width] + x(2);
          patch(spring_x, spring_z, [0.4, 0.4, 0.4]);
      end
      
      p = r*base;
      patch(x(1)+p(1,:), x(2)+p(2,:),1+0*p(1,:),'b','FaceColor',[0 0 0])
      
      p = r*[obj.L+pin(1,:);pin(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
      p = r*[-obj.L+pin(1,:);pin(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 0]);
      
      p = r*[obj.L+prop(1,:);prop(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
      p = r*[-obj.L+prop(1,:);prop(2,:)];
      patch(x(1)+p(1,:),x(2)+p(2,:),0*p(1,:),'b','FaceColor',[0 0 1]);
                 
      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-2.0 2.0 -1.0 1.0]);
      drawnow;
    end    
  end
  
end
