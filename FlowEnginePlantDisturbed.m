classdef FlowEnginePlantDisturbed < DrakeSystem

  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - pressure   [N/cm^2 = 1e4 Pa = 10 kPa]
  % input:
  %  u(1) - mass flow in
  %  u(2) - disturbance input (ground level changing)

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    
    %bar rocket properties
    %M = 9;      % mass of air skate [kg] 
    %M = 250;      % mass of air skate [kg] 
    M = 909/4;
    
    
    %length/width/area
    L = 0.9144; % length of skate [m]
    W = 0.3048; % width  of skate [m]
    A;          % skate area [m^2]
    Per;        % skate perimeter [m]
    
    %z coordinate stuff
    h_skate = 0.01905; %height of hdpe bezel [m]
    %h_bag   =  0.0015;  %bag thickness when fully inflated [m]
    h_bag = 1e-5;
    %ride_height = 0.000635; %steady state ride height [m] (0.25")
    %ride_height = 0.001;
    ride_height = 2.4e-4;
    
    %environment properties
    g = 9.81;   % gravity [N]
    b = 0;      %friction [N/(m/s)]
    pa = 101325; % atmospheric pressure [N/m^2=Pa]
    rhoa = 1.177; %atmospheric density [kg/m^3]
    gamma = 1.40;   %specific heat ratio at stp
    
    T = 300; %300 Kelvin
    R = 287; %Specific gas constant of air, [J/(kg*K)]
    
    %controller balance
    delta = 10; %compromise between Q and R, R = delta*R0
    
    x0;
    u0;
  end
  
  methods
    function obj = FlowEnginePlantDisturbed()
      obj = obj@DrakeSystem(3, ... %number of continuous states
                                  0, ... %number of discrete states
                                  2, ... %number of inputs
                                  3, ... %number of output
                                  false, ... %because the output does not depend on u
                                  true); %because the dynamics and output do not depend on t      
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    
      %deal with input limits
      obj = obj.setInputLimits(0, Inf);
      
      %fill in the rest of the properties      
      obj.A = obj.L*obj.W; %skate area [m^2]
      obj.Per = 2*(obj.L + obj.W); %skate perimeter [m]

      
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        z0 = obj.ride_height;
        
        %input equilibrium
        %find bag pressure at equilibrium
        p0 = obj.M*obj.g/obj.A + obj.pa; %equilibrium bag pressure [N/cm^2]        
        
%         %from polytropic pressure/density relationship, find bag density
%         rho0 = obj.rhoa*(p0/obj.pa)^(1/obj.gamma);
%         
%         %use bag density to get outlet velocity at equilibrium
%         vout0 = sqrt((2*obj.gamma)/(obj.gamma-1) * (p0/rho0 - obj.pa/obj.rhoa));
%         
%         %at equilibrium, mass flow in = mass flow out
%         Ae = 2*(obj.W + obj.L)*z0; %air exit area, perimeter*z
%         
%         min0 = obj.rhoa * Ae * vout0;
%         
        min0 = mass_flow_out( p0, obj.pa, z0,  obj.gamma, obj.Per, obj.R, obj.T );
        
        x0 = [z0; 0; p0];        
        u0 = [min0; 0];
    end
    
    function xdot = dynamics(obj,~,x,u)
      %dynamics for the air cushion
      %time invariant system
      %horrifically ugly though  
        if u >= 0
            u_thresh = u;
        else
            u_thresh = 0;
        end
        
       % u_thresh = u;
%       
%         rho = obj.rhoa*power(x(3)/obj.pa,(1/obj.gamma));
%         vout = sqrt((2*obj.gamma)/(obj.gamma-1) * (x(3)/rho - obj.pa/obj.rhoa));
% 
%         Ae = 2*(obj.W + obj.L) * x(1);
%         mout = obj.rhoa * Ae * vout;
%         
        mout = mass_flow_out( x(3), obj.pa, x(1) + u(2),  obj.gamma, obj.Per, obj.R, obj.T );
        
        %V = obj.A * x(1);
        
        outside = obj.gamma*obj.R*obj.T/(obj.A * (x(1) + u(2) + obj.h_bag));
        inside = u(1) - mout - x(3)*obj.A*x(2)/(obj.R*obj.T);
                
        
        xdot = [x(2);
                %(u - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
                (x(3) - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
                outside*inside];
    end
    
    function y = output(obj,~,x,u)
      % full state feedback is allowed here
      y = x;
    end

    
    function x_init = getInitialState(obj)
      z_init = obj.ride_height + 0.0001;  
      p_init = obj.x0(3);
      %x = [z_init - obj.ride_height;  %initial z position (ride height)
      x_init = [z_init;
           0;  %initial z speed
           p_init]; %initial bag pressure
    end
    
    function [c,V] = hoverLQR(obj, ROA)      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      Q = diag([1 1 0.1]);
      R = diag([1 1])*obj.delta;

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
