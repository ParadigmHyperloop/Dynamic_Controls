classdef FlowEnginePlantPressure < DrakeSystem

  % state (world coordinates):  
  %  x(1) - z position [m]
  %  x(2) - z speed    [m/s]
  %  x(3) - pressure   [N/m^2 = Pa]
  % input:
  %  u(1) - mass flow int

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    
    %bar rocket properties
    M = 9;      % mass of air skate [kg] 
    
   %length/width/area
    L = 0.9144; % length of skate [m]
    W = 0.3048; % width  of skate [m]
    A;
    
    %z coordinate stuff
    h_skate = 0.01905; %height of hdpe bezel [m]
    h_bag   =  0.0015;  %bag thickness when fully inflated [m]
    ride_height = 0.00635; %steady state ride height [m] (0.25")
    
    %environment properties
    g = 9.81;   % gravity [N]
    b = 0;      %friction [N/(m/s)]
    pa = 101325; % atmospheric pressure [N/m^2=Pa]
    rhoa = 1.225; %atmospheric density [kg/m^3]
    gamma = 1.40;   %specific heat ratio at stp
    
    T = 300; %300 Kelvin
    R = 287; %Specific gas constant of air, [J/(kg*K)]
    
    %controller balance
    delta = 0.0001; %compromise between Q and R, R = delta*R0
     
    x0;
    u0;
  end
  
  methods
    function obj = FlowEnginePlantPressure()
      obj = obj@DrakeSystem(3, ... %number of continuous states
                                  0, ... %number of discrete states
                                  1, ... %number of inputs
                                  3, ... %number of output
                                  false, ... %because the output does not depend on u
                                  true); %because the dynamics and output do not depend on t      
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    
      %deal with input limits
      obj = obj.setInputLimits(0, Inf);
      
      %fill in the rest of the properties      
      obj.A = obj.L*obj.W; %skate area [cm^2]
      
            %obj.p0 = obj.M*obj.g;
      
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        x0 = [obj.ride_height; 0; 0];
        
        %input equilibrium
        p0 = obj.M*obj.g/obj.A + obj.pa ; %equilibrium bag pressure [N/cm^2]        
        
        u0 = p0;
    end
    
    function xdot = dynamics(obj,~,x,u)
      %dynamics for the air cushion
      %time invariant system
      %horrifically ugly though
      
%       xdot = [0; 0; 0];
%       
%       
%       xdot(1) = zd; %zd
%       
%       xdot(2) = u/obj.M - obj.g;
%       %xdot(2) = obj.A/obj.M * (u - obj.p0) - obj.g - obj.b*zd;  %zdd
        xdot = [x(2);
                (u - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
                
                -x(3)]; %placeholder on pressure update
      
    end
    
    function y = output(obj,~,x,u)
      % full state feedback is allowed here
      y = x;
    end

    
    function x = getInitialState(obj)
      z_init = 0.03;  
      %z_init = 0.000;  
      
      %x = [z_init - obj.ride_height;  %initial z position (ride height)
      x = [z_init;
           0;  %initial z speed
           0]; %initial bag pressure
    end
    
    function [c,V] = hoverLQR(obj, ROA)      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      Q = diag([10 1 1]);
      R = diag(1)*obj.delta;

      if ROA
        [c,V0] = tilqr(obj,x0,u0,Q,R);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        V=regionOfAttraction(pp,V0,options);
      else
        [c, V0] = tilqr(obj,x0,u0,Q,R);
        V = V0;
      end
    end
    
  end
  
end
