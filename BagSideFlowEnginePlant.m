classdef BagSideFlowEnginePlant < DrakeSystem

  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]  
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - underneath skate pressure [Pa]
  %  x(5) - side of skate pressure [Pa]
  % input:
  %  u(1) - mass flow into bag
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
    %M_total = 1000;
    %M_total = 324.319;
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
    
    %side/full pod area
    L_side = 2;  % pod length [m]
    W_side = 1;  % pod length [m]
    A_side;      % pod area [m^2]
    Per_side       % pod perimeter [m]
    
    %bag property
    porosity = 0.01; %How porous the bag is
    %porosity = 1; %How porous the bag is
    
    
    %z coordinate stuff
    h_skate = 0.0445; %height of hdpe bezel [m]
    %h_bag   =  0.0015;  %bag thickness when fully inflated [m]
    %h_bag = 1e-5;
    %h_bag = 2e-4;
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
    %input_flow = 10;
    
    
    select_ride_height = 0;
    
    %dimensionless quantities
    Mbar;
    Perbar; %Lbar
    Minbar; 
    u0_scfm;
    
    %environment/air properties
    g = 9.81;   % gravity [N]
    b = 0;      %friction [N/(m/s)]
    
    %pa = 101325.0; % atmospheric pressure [N/m^2=Pa]
    pa = 5 * 133.322; %tube pressure torr->pascal [N/m^2=Pa]
    
    rhoa; %atmospheric density [kg/m^3]
    gamma = 1.40;   %specific heat ratio at stp
    
    T = 300; %300 Kelvin
    R = 287; %Specific gas constant of air, [J/(kg*K)]
    mu = 1.85e-5; %Dynamic Viscosity of Air at 300K [Pa*s]
    
    %controller balance
    delta = 10; %compromise between Q and R, R = delta*R0
    
    x0;
    u0;
  end
  
  methods
    function obj = BagSideFlowEnginePlant()
      obj = obj@DrakeSystem(5, ... %number of continuous states
                                  0, ... %number of discrete states
                                  1, ... %number of inputs
                                  5, ... %number of output
                                  false, ... %because the output does not depend on u
                                  true); %because the dynamics and output do not depend on t      
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    
      %deal with input limits
      obj = obj.setInputLimits(0, Inf);
      
      
      
      %fill in the rest of the properties  
      obj.rhoa = obj.pa/(obj.R * obj.T);
      
      
      obj.M = obj.M_total;
      
      %geometric properties of the skate/pod
      obj.A_skate = obj.L*obj.W; %skate area [m^2]
      obj.Per_skate = 2*(obj.L + obj.W); %skate perimeter [m]
      obj.A = obj.porosity * obj.A_skate * obj.N;
      obj.Per = obj.Per_skate * obj.N;
      
      obj.A_side = obj.L_side*obj.W_side;
      obj.Per_side = 2*(obj.L_side + obj.W_side);
      
      obj.h = obj.h_bag + obj.h_skate;
      %obj.h = obj.h_bag;
      obj.V = obj.A*obj.h;
      
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
     
      obj.u0_scfm = obj.u0 * 1800.24;
      

    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        
%         %find pressure
%         p0 = obj.pa + obj.M*obj.g/obj.A;
%         pb0 = obj.mu * obj.h_bag/(obj.porosity * obj.A^2 * obj.rhoa) + p0;        
% 
%         %find ride height and mass flow in
%         if obj.select_ride_height
%             %find flow from ride height
%             z0 = obj.ride_height;
%             min0 = mass_flow_out( p0, obj.pa, z0,  obj.gamma, obj.Per, obj.R, obj.T );
%             obj.input_flow = min0;
%         else
%             %find ride height from flow
%             min0 = obj.input_flow;
%             z_balance = @(z) mass_flow_out(p0, obj.pa, z,  obj.gamma, obj.Per, obj.R, obj.T ) - min0;
%             z0 = fzero(z_balance, obj.ride_height);
%             obj.ride_height = z0;
%         end
%         
%                
%         x0 = [z0; 0; pb0; p0];        
%         u0 = min0;              
%         
%         %export equilibrium point
%         x0 = [z0; 0; pb0; p0];        
%         u0 = min0;

        %this stuff is a bit trickier than usual.
        %system of three nonlinear equations
        min0 = obj.input_flow;
        
        %for f:
        %   x(1): z coordinate, ride height
        %   x(2): under pressure
        %   x(3): side pressure
        
%         f = @(z, p_under, p_side) [ p_under * obj.A + p_side * (obj.A_side - obj.A) - obj.M*obj.g; ...
%             min0 - mass_flow_out(p_under, p_side, z, obj.gamma, obj.Per, obj.R, obj.T); ...
%             min0 - mass_flow_out(p_side, obj.pa, z + obj.h_bag, obj.gamma, obj.Per_skate, obj.R, obj.T)];
%         
        f = @(x) [ x(2) * obj.A + x(3)* (obj.A_side - obj.A) - obj.M*obj.g; ...
            min0 - mass_flow_out(x(2), x(3), x(1), obj.gamma, obj.Per, obj.R, obj.T); ...
            min0 - mass_flow_out(x(3), obj.pa, x(1) + obj.h_bag, obj.gamma, obj.Per_skate, obj.R, obj.T)];
        
        state_0 = fsolve(f, [obj.ride_height; obj.pa+100; obj.pa]);         
        %[z0, p_under0, p_side0] = state_0;
        z0 = state_0(1);
        p_under0 = state_0(2);
        p_side0 = state_0(3);
        %[z0, p_under0, p_side0] = fsolve(f, [obj.ride_height; obj.pa+1000; obj.pa]);
        
        p_bag0 = obj.p_under0 + obj.mu * obj.h_bag/(obj.porosity * obj.A^2 * obj.rhoa);
        
        x0 = [z0, 0, p_bag0, p_under0, p_side0];
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
        
       %source to bag
       min = u(1);
       
       %lift force exerted by pressurized air
       lift_force = (x(4)-obj.pa) * obj.A + (x(5)-obj.pa) * (obj.A_side - obj.A);
       
       %flow into bag from source
       %darcy flow
       mbag = obj.porosity*obj.A^2 * (x(3) - x(4))/(obj.mu * obj.h_bag);       
       p_bag_outside = (obj.gamma*obj.R*obj.T)/(obj.A*obj.h_skate);       
       p_bag_inside = min - mbag;
              
       %bottom to side
       mescape = mass_flow_out( x(4), x(5), x(1),  obj.gamma, obj.Per, obj.R, obj.T );                            
       p_under_inside =  (obj.gamma*obj.R*obj.T)/(obj.A*x(1));       
       p_under_outside = mbag - mescape - x(4)*obj.A*x(2)/(obj.R*obj.T);
       
       
       %side to outside
       mout = mass_flow_out( x(5), obj.pa, x(1),  obj.gamma, obj.Per_side, obj.R, obj.T );                            
       p_side_inside =  (obj.gamma*obj.R*obj.T)/((obj.A_side - obj.A)*(x(1) + obj.h_skate));       
       p_side_outside = mescape - mout - x(4)*(obj.A_side - obj.A)*x(2)/(obj.R*obj.T);
       
       
       %dynamical evolution of system
       xdot = [x(2);
               %(x(4) - obj.pa)*obj.A/obj.M - obj.g - obj.b*x(2);
               lift_force/obj.M - obj.g;
               p_bag_inside   * p_bag_outside;
               p_under_inside * p_under_outside;
               p_side_inside  * p_side_outside];
           
    end
    
    function y = output(obj,~,x,u)
      % full state feedback is allowed here
      y = x;
    end

    
    function x_init = getInitialState(obj)
      z_init = obj.ride_height + 0.0001;  
      pb_init = obj.x0(3);
      p_init = obj.x0(4);
      x_init = [z_init;
           0;       %initial z speed
           pb_init; %initial bag pressure
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
