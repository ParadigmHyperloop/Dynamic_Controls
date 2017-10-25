classdef FlowEnginePlant < DrakeSystem

  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - pressure   [N/cm^2 = 1e4 Pa = 10 kPa]
  % input:
  %  u(1) - mass flow int

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    
    %bar rocket properties
    %M = 9;      % mass of air skate [kg] 
    %M = 250;      % mass of air skate [kg] 
    
%     %Pod 2
%     M = 909/4;
%         
%     %length/width/area
%     L = 0.9144; % length of skate [m]
%     W = 0.3048; % width  of skate [m]
    
    %Pod 3
    N = 4;
    %M_total = 324.319;
    M_total = 909;
    %M_total = 4.5;
    M;
        
    %length/width/area
    L = 0.9144; % length of skate [m]
    W = 0.3048; % width  of skate [m] Pod 2
    %W = 0.127; % width  of skate [m]
    %W = 0.9144;
    
    %L = 0.1524;
    %W = 0.2032;
    
    A;          % skate area [m^2]
    A_skate;    % per skate area
    Per;        % skate perimeter [m]
    Per_skate;  % per skate perimeter
    V;          % skate volume [m^3]
    
    %bag property
    %porosity = 0.04; %How porous the bag is
    porosity = 1; %How porous the bag is
    
    
    %z coordinate stuff
    h_skate = 0.0445; %height of hdpe bezel [m]
    %h_bag   =  0.0015;  %bag thickness when fully inflated [m]
    %h_bag = 1e-5;
    %h_bag = 0.2;
    h_bag = 0.020066;
    h;
    %ride_height = 0.000635; %steady state ride height [m] (0.25")
    %ride_height = 0.001;
    %ride_height = 1.2e-4;
    %ride_height = 1.52e-4; %small width
    %ride_height  = 2.01e-4; %large width 
    ride_height = 6.38e-4;
    %ride_height = 5e-3;
    
    %mass flow in if desired
    input_flow = 0.1600;
    %input_flow = 0.005;
    
    select_ride_height = 0;
    
    %dimensionless quantities
    Mbar;
    Perbar; %Lbar
    Minbar; 
    u0_scfm;
    
    %environment properties
    g = 9.81;   % gravity [N]
    b = 0;      %friction [N/(m/s)]
    pa = 101325.0; % atmospheric pressure [N/m^2=Pa]
    %pa = 5 * 133.322; %tube pressure torr->pascal [N/m^2=Pa]
    rhoa; %atmospheric density [kg/m^3]
    gamma = 1.40;   %specific heat ratio at stp
    
    T = 300; %300 Kelvin
    R = 287; %Specific gas constant of air, [J/(kg*K)]
    
    %controller balance
    delta = 100; %compromise between Q and R, R = delta*R0
    
    x0;
    u0;
  end
  
  methods
    function obj = FlowEnginePlant()
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
      %obj.M = obj.M_total / obj.N;
      obj.rhoa = obj.pa / (obj.R*obj.T);
      
      obj.M = obj.M_total;
      obj.A_skate = obj.L*obj.W; %skate area [m^2]
      obj.Per_skate = 2*(obj.L + obj.W); %skate perimeter [m]
      obj.A = obj.porosity * obj.A_skate * obj.N;
      obj.Per = obj.Per_skate * obj.N;
      
      obj.h = obj.h_bag + obj.h_skate;
      %obj.h = obj.h_bag;
      obj.V = obj.A*obj.h;
      
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
      
      %dimensionless characterization
      obj.Perbar = obj.Per/obj.h;
      obj.Mbar = obj.M/(obj.rhoa * obj.h^3);
      
      
      v0 = obj.u0/(obj.rhoa * obj.Per * obj.x0(1));
      obj.Minbar = obj.u0/(obj.rhoa * v0 * obj.h^2);
      
      obj.u0_scfm = obj.u0 * 1800.24;
      

    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        
        %find bag pressure at equilibrium
        p0 = obj.M*obj.g/obj.A + obj.pa; %equilibrium bag pressure [N/cm^2]        
        
        
        if obj.select_ride_height
            %find flow from ride height
            z0 = obj.ride_height;
            min0 = mass_flow_out( p0, obj.pa, z0,  obj.gamma, obj.Per, obj.R, obj.T );
            obj.input_flow = min0;
        else
            %find ride height from flow
            min0 = obj.input_flow;
            z_balance = @(z) mass_flow_out(p0, obj.pa, z,  obj.gamma, obj.Per, obj.R, obj.T ) - min0;
            z0 = fzero(z_balance, obj.ride_height);
            obj.ride_height = z0;
        end
        
        x0 = [z0; 0; p0];        
        u0 = min0;
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
        mout = obj.porosity * mass_flow_out( x(3), obj.pa, x(1),  obj.gamma, obj.Per, obj.R, obj.T );
        
        %V = obj.A * x(1);
        outside =  (obj.gamma*obj.R*obj.T)/(obj.A*(x(1) + obj.h));
        net_mass_flow = u(1) - mout;
        inside = net_mass_flow - x(3)*obj.A*x(2)/(obj.R*obj.T);
        
        xdot = [x(2);
                %(u - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
                (x(3) - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
                
                %(obj.gamma*obj.R*obj.T)/(obj.A*(x(1) + obj.h_bag)) * (u_thresh - mout) - (obj.gamma*x(2)/(x(1)+obj.h_bag))* x(3)]; %placeholder on pressure update
                %(obj.gamma*obj.R*obj.T)/(obj.A*(x(1) + obj.h)) * (u_thresh - mout) - (obj.gamma*x(2)/(x(1)+obj.h_bag + obj.h_skate))* x(3)]; %placeholder on pressure update
                outside * inside];
                
      
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
