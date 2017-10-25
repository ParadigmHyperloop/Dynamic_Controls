classdef FlowEngineVisualizer < Visualizer
% Implements the draw function for the Flow Engine model
% based on heave dynamics of an air cushioned vehicle

  properties
    h_skate;
    h_bag;
    ride_height; %steady state ride height [m]
    
    
    W; % width  of skate [cm]
  end

  methods
    function obj = FlowEngineVisualizer(plant)
      typecheck(plant,'FlowEnginePlant') || typecheck(plant,'FlowEnginePlantPressure') || typecheck(plant,'BagFlowEnginePlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      
      obj.h_skate = plant.h_skate;
      obj.h_bag = plant.h_bag;
      obj.ride_height = plant.ride_height;
      obj.W = plant.W;
    end
    
    function draw(obj,t,x)
      % Draw the flow engine.
      persistent hFig skate bag;

      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        %base = [1.2*obj.L*[1 -1 -1 1]; .025*[1 1 -1 -1]];
        skate = [obj.W/2*[1 -1 -1 1]; (obj.h_skate)*[0 0 1 1]];
        bag   = [obj.W/2*[1 -1 -1 1]; (obj.h_bag)*[0 0 1 1]];
        

      end
            
      sfigure(hFig); cla; hold on; view(0,90);
            
      %patch(x(1)+p(1,:), x(2)+p(2,:),1+0*p(1,:),'b','FaceColor',[0 0 0])
      
      %skate patch
      axis image; axis([-0.2 0.2 -0.01 0.30]);
      plot(xlim, [0, 0], 'k');
      plot(xlim, [obj.ride_height, obj.ride_height], '--k')      
      
      patch(bag(1, :),   x(1) - bag(2, :), [0.8, 0, 0], 'EdgeColor','none')
      patch(skate(1, :), x(1) + skate(2, :), [1, 1, 1]*0.4,'EdgeColor','none')
      
      title(['t = ', num2str(t(1),'%.2f') ' sec']);
      %set(gca,'XTick',[],'YTick',[])
      xlabel('skate width (m)')
      ylabel('height (m)')
      
      
      drawnow;
    end    
  end
  
end
