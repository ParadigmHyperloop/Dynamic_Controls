classdef AirSpringPlant1d < DrakeSystem

%bag is in full contact with the ground
%purely heave motion, no horizontal travel
    
  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - underneath skate pressure [Pa]
  % input:
  %  u(1) - mass flow into bag

  properties  
    %Pod 3
    N = 4;

    M_total = 909;
    M;
        
    %length/width/area
    L = 0.9144; % length of skate [m]
    W = 0.3048; % width  of skate [m] Pod 2
    %W = 0.127; % width  of skate [m]
    %W = 0.9144;
    
    A;          % skate area [m^2]
    A_skate;    % per skate area
    Per;        % skate perimeter [m]
    Per_skate;  % per skate perimeter
    V;          % skate volume [m^3]
    
    %bag property
    permeability = 1e-8; %How porous the bag is
    
    %z coordinate stuff
    h_skate = 0.0445; %height of hdpe bezel [m]
    %h_bag = 2e-4;
    h_bag = 0.020066;
    h;
    
    ride_height = 6.38e-4;
    
    %mass flow in if desired
    input_flow = 0.1600;
    %input_flow = 1;
    
    select_ride_height = 0;
    
    %dimensionless quantities
    Mbar;
    Perbar; %Lbar
    Minbar; 
    u0_scfm;
    
    %environment/air properties
    g = 9.81;   % gravity [N]
    b = 0;      %friction [N/(m/s)]
    
    pa = 101325.0; % atmospheric pressure [N/m^2=Pa]
    %pa = 5 * 133.322; %tube pressure torr->pascal [N/m^2=Pa]
    
    rhoa; %atmospheric density [kg/m^3]
    gamma = 1.40;   %specific heat ratio at stp
    
    T = 300; %300 Kelvin
    R = 287; %Specific gas constant of air, [J/(kg*K)]
    mu = 1.85e-5; %Dynamic Viscosity of Air at 300K [Pa*s]
    darcy_factor; %mass flow = darcy_factor * pressure difference
    
    
    %controller balance
    delta = 10; %compromise between Q and R, R = delta*R0
    
    x0;
    u0;
  end
  
  methods
    function obj = AirSpringPlant1d()
      obj = obj@DrakeSystem(3, ... %number of continuous states
                                  0, ... %number of discrete states
                                  1, ... %number of inputs
                                  3, ... %number of output
                                  false, ... %because the output does not depend on u
                                  true); %because the dynamics and output do not depend on t      
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    
      %deal with input limits
      %obj = obj.setInputLimits(0, Inf);
      
      
      
      %fill in the rest of the properties  
      obj.rhoa = obj.pa/(obj.R * obj.T);
      
      
      obj.M = obj.M_total;
      obj.A_skate = obj.L*obj.W; %skate area [m^2]
      obj.Per_skate = 2*(obj.L + obj.W); %skate perimeter [m]
      obj.A = obj.A_skate * obj.N;
      obj.Per = obj.Per_skate * obj.N;
      
      obj.h = obj.h_bag + obj.h_skate;
      
      obj.darcy_factor = obj.rhoa * obj.permeability*obj.A /(obj.mu * obj.h_bag);
      
      %now find equilibrium point
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
      obj.ride_height = obj.x0(1);
      obj.input_flow = obj.u0;
      
      obj.u0_scfm = obj.u0 * 1800.24;
      

    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        
        %find pressure
        p0 = obj.pa + obj.M*obj.g/obj.A;        
        
        %ride height from darcy flow
        
        %find ride height and mass flow in
        if obj.select_ride_height
            %find flow from ride height
            z0 = obj.ride_height;
            min0 = obj.permeability*obj.Per * z0/(obj.mu * obj.h_skate) * (p0 - obj.pa);      
            obj.input_flow = min0;
        else
            %find ride height from flow
            min0 = obj.input_flow;
            z0 = min0 * obj.mu * obj.h_skate/(obj.rhoa * obj.permeability * obj.Per * (p0 - obj.pa));             obj.ride_height = z0;
            obj.ride_height = z0;
        end
%         
%         %pb0 = p0 + min0 * obj.mu * obj.h_bag/(obj.permeability * obj.A^2 * obj.rhoa);
       
        
        %export equilibrium point
        x0 = [z0; 0; p0];        
        u0 = min0;
    end
    
    function xdot = dynamics(obj,t,x,u)
      %dynamics for the air cushion
      %time invariant system
      %horrifically ugly though  
       if u >= 0
           u_thresh = u;
       else
           u_thresh = 0;
       end 
       
       %incoming mass flow rate
       min = u_thresh;
       %min = obj.u0;
       
       
       %darcy flow (from bag to outside
       Ae = obj.Per * x(1); %exit area of bag, since the bag does not let out air on bottom
       mescape = obj.permeability*Ae/(obj.mu * obj.h_skate) * (x(3) - obj.pa);      
       
       %pressure in the bag
       p_outside = (obj.gamma*obj.R*obj.T)/(obj.A*x(1));
       p_inside  = min - mescape - x(3)*obj.A*x(2)/(obj.R*obj.T);
       
       %pressure gradient force
       %pb-pa * area
       lift_force = (x(3) - obj.pa) * obj.A;
       
       %dynamical evolution of system
       xdot = [x(2);
               lift_force/obj.M - obj.g;
               p_outside * p_inside];
           
    end
    
    function y = output(obj,~,x,u)
      % full state feedback is allowed here
      y = x;
    end

    
    function x_init = getInitialState(obj)
      z_init = obj.ride_height + 1e-7;  
      p_init = obj.x0(3);
      x_init = [z_init;
                0;       %initial z speed
                p_init]; %initial bottom pressure
    end
    
    function [c,V] = hoverLQR(obj, ROA)      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      Q = diag([1 1 1e-2 1e-2]);
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