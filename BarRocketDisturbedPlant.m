classdef BarRocketDisturbedPlant < SecondOrderSystem

  % state (world coordinates):  
  %  q(1) - x position
  %  q(2) - z position
  %  q(3) - pitch (theta)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    
    %bar rocket properties
    L = 0.25;       % length of rotor arm
    m = 0.486;      % mass of quadrotor
    I = 0.00383;    % moment of inertia
    
    %environment properties
    g = 9.81;       % gravity      
    %kx = 1;       %spring
    kx = 0;
    %b = [1; 1; 0.1]*1; %friction
    b = [0; 0; 0];
    
    
%     %horizontal cross-wind
%     %bounded between [-w_max, w_max]
     w_sigma = 0.25;
%     %w_max = 5*w_sigma;
     w_max = 0.05;
%     %w_base = makedist('Normal', 'mu', 0, 'sigma', w_sigma);
%     w_dist = truncate(makedist('Normal', 'mu', 0, 'sigma', w_sigma), -1, 1);
%     
    %Delta = makedist
    
    %Bw = [1; 0; 0]*w_max;
    %Bw = [0; 0; 0];
    w_dist;
    Bw;
    
    %Equilibrium Position
    x0;
    u0;
  end
  
  methods
    function obj = BarRocketPlant()
      obj = obj@SecondOrderSystem(3,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
      
      %equilibrium system
      obj.x0 = Point(obj.getStateFrame,zeros(6,1));
      obj.u0 = Point(obj.getInputFrame,obj.m*obj.g/2 * [1;1]);
      
      %noise
      obj.w_dist = truncate(makedist('Normal', 'mu', 0, 'sigma', obj.w_sigma), -1, 1);
      obj.Bw = [1; 0; 0]*obj.w_max;
    end
    
    function qdd = sodynamics(obj,~,q,qd,u)
      %Implement the second-order dynamics for the bar rocket
      %time invariant system
      %f(x) + g(x) u
      %nonlinearity affine in control
      
      
      %force term
      mass_inertia = [obj.m; obj.m; obj.I];
     
      %g
      force_response = [-sin(q(3)), -sin(q(3));
            cos(q(3)),  cos(q(3));
            -obj.L/2,   obj.L/2];
      

      spring   = [-obj.kx; 0; 0].*q;
      friction = -obj.b'*qd;
      gravity  = [0; -obj.m*obj.g; 0];
      
      force = spring + friction + gravity;
      gu = force_response*u;
      
      %cross-wind
      %w = obj.Bw * random(obj.w_dist, 1);
      w = [0; 0; 0];
      %w = [0; -0.05; 0];
      
      qdd = (force + gu + w) ./ mass_inertia;
    end
    
    function x = getInitialState(obj)
      x = randn(6,1);
      %x = [0.8; -0.5; 0.5; 0; 0; 0];
    end
    
    function [c,V] = hoverLQR(obj)
      Q = diag([10 10 10 1 1 (obj.L/2/pi)]);  %Q = diag([10*ones(1,3) ones(1,3)]);
      R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

      if (nargout>1)
        [c,V0] = tilqr(obj,obj.x0,obj.u0,Q,R);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,obj.x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        %options.method='bilinear';
        %options.degV=4;
        V=regionOfAttraction(pp,V0,options);
      else
        c = tilqr(obj,obj.x0,obj.u0,Q,R);
      end
    end
    
    function [c, V] = hoverHinf(obj)
      obj.x0 = Point(obj.getStateFrame,zeros(6,1));
      obj.u0 = Point(obj.getInputFrame,obj.m*obj.g/2 * [1;1]);
      Q = diag([10 10 10 1 1 (obj.L/2/pi)]);  %Q = diag([10*ones(1,3) ones(1,3)]);
      R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

      gamma = 10;
      
      if (nargout>1)
        [c,V0] = tiHinf(obj,obj.x0,obj.u0,Q,R, obj.Bw, gamma);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,obj.x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        %options.method='bilinear';
        %options.degV=4;
        V=regionOfAttraction(pp,V0,options);
      else
        c = tiHinf(obj,obj.x0,obj.u0,Q,R, obj.Bw, gamma);
      end
    end
    
  end
  
end
