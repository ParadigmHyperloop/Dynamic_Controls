classdef AirSpringPlant1d_disturbed_2 < DrakeSystem

%bag is in full contact with the ground
%purely heave motion, no horizontal travel
    
  % state (world coordinates):  
  %  x(1) - z position [m]
  %  x(2) - z speed    [m/s]
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - solenoid response [single pole filter]
  % input:
  %  u(1) - mass flow into bag
  %  u(2) - DISTURBANCE (change in track height) [m]
  %  u(3) - DISTURBANCE (length of gaps in track) [m]

  properties  
    %Pod 3
    N = 4;
    pressure_scale = 1e-5;
    %pressure_scale = 1;

    M_total = 909; %kg
    M;
        
    %length/width/area
    L = 0.9144; % length of skate [m] 36"
    W = 0.3048; % width  of skate [m] 12"
    
    %W = 0.127; % width  of skate [m]
    %W = 0.9144;
    
    A;          % skate area [m^2]
    A_skate;    % per skate area
    Per;        % skate perimeter [m]
    Per_skate;  % per skate perimeter
    V;          % skate volume [m^3]
    
    %bag property
    darcy_flow= 0;
    permeability = 1e-5; %How porous the bag is
    %forchheimer_beta = 1e8; % forchheimer coefficient[1/m]
    forchheimer_beta = 1.2e6; % forchheimer coefficient[1/m]
    
    %z coordinate stuff
    h_skate = 0.0445; %height of hdpe bezel [m]
    %h_bag = 2e-4;
    h_bag = 0.020066;
    h;
    
    bag_thickness = 2e-3; %bag thickness is 0.079"
    %bag_thickness = 1e-2; %bag thickness is 0.079"
    
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
    
    
    %controller balance
    delta = 10; %compromise between Q and R, R = delta*R0
    
    solenoid_frequency = 2*pi*65; %65 hz
    
    x0;
    u0;
  end
  
  methods
    function obj = AirSpringPlant1d_disturbed_2()
      obj = obj@DrakeSystem(4, ... %number of continuous states
                                  0, ... %number of discrete states
                                  3, ... %number of inputs
                                  4, ... %number of output
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
           
      obj.pa = obj.pa * obj.pressure_scale;
      
      %now find equilibrium point
      [obj.x0, obj.u0] = obj.find_equilibrium_point();
      obj.ride_height = obj.x0(1);
      obj.input_flow = obj.u0(1);
      
      obj.u0_scfm = obj.u0 * 1800.24;
      

    end
    
    function [x0, u0] = find_equilibrium_point(obj)
        %state equilibrium
        
        %find pressure
        p0 = obj.pa + obj.pressure_scale * obj.M*obj.g/obj.A;        
        %p = p0 / obj.pressure_scale;
        dp = (p0 - obj.pa) / obj.pressure_scale;
        %ride height from darcy flow
        
        %find ride height and mass flow in
        if obj.select_ride_height
            %find flow from ride height
            z0 = obj.ride_height;
            min0 = obj.permeability*obj.Per * z0/(obj.mu * obj.bag_thickness) * (p0 - obj.pa)/obj.pressure_scale;      
            obj.input_flow = min0;
        else
            %find ride height from flow
            min0 = obj.input_flow;
            %z0 = min0*obj.mu*obj.bag_thickness/(obj.permeability * obj.Per * (p0-obj.pa));

            %quadratic form of forchheimer equation
            %a = obj.forchheimer_beta/obj.Per^2;
            %b = obj.mu/(obj.permeability * obj.Per);

            a = obj.forchheimer_beta/obj.Per^2;
            b = obj.mu/(obj.permeability * obj.Per);            
            c = -dp/obj.bag_thickness * obj.rhoa;

            %        a = obj.forchheimer_beta/obj.Per^2;
            %        b = obj.mu/(obj.permeability * obj.Per);            
            %        %c = -dp/obj.bag_thickness * obj.rhoa;
            %        c = dp/obj.bag_thickness * obj.rhoa;

            if obj.darcy_flow
               min_over_z = -c/b;
            else
               min_over_z = (-b + sqrt(b^2 - 4*a*c))/(2*a);
            end
            
            z0 = min0 / min_over_z;            
            
            obj.ride_height = z0;
            
        end
%         
%         %pb0 = p0 + min0 * obj.mu * obj.h_bag/(obj.permeability * obj.A^2 * obj.rhoa);
       
        
        %export equilibrium point
        x0 = [z0; 0; p0; min0];        
        u0 = [min0; 0; 0];
    end
    
    function xdot = dynamics(obj,t,x,u)
      %dynamics for the air cushion
      %time invariant system
      %horrifically ugly though  
      if u(1) >= 0
          u_thresh = u(1);
      else
          u_thresh = 0;
      end 
      
      p = x(3) / obj.pressure_scale;
      dp = (x(3) - obj.pa) / obj.pressure_scale;      
      
      %incoming mass flow rate
       
      %active control
      w_track_height = u(2);
      w_track_gap = u(3);
       
      %open loop control
      %u_thresh = obj.u0(1);
      %w_track_height = 0;
      %w_disturbance = -5e-3;
      %w_track_gap = 0
       
     eff_height = x(1) - w_track_height;       
     %height spikes, when there's not enough clearance
     if eff_height < 0
         spike = 1e6;
     else
         spike = x(2);
     end
     
     %darcy flow (from bag to outside)
     %the effective area is the area in which air can escape the bag
     %when the skate is hovering over the gap in the track, the effective
     %area is increased, as air can fall out the slot.
     
     Ae0 = obj.Per*eff_height;
     Ae = Ae0 + (obj.W*obj.N)*w_track_gap;
     
     %pressure gradient across the bag thickness
     dpdx = dp/obj.bag_thickness;

     a = obj.forchheimer_beta;
     b = obj.mu/obj.permeability;            
     c = - dpdx * obj.rhoa;
     
     if obj.darcy_flow
         min_over_Ae = -c/b;
     else
         min_over_Ae = (-b + sqrt(b^2 - 4*a*c))/(2*a);
     end

     %effective area      
     mescape = Ae * min_over_Ae;
       
     %pressure change in the bag
     p_outside = (obj.gamma*obj.R*obj.T)/(obj.A*eff_height)* obj.pressure_scale;
     p_inside  = x(4) - mescape - p*obj.A*x(2)/(obj.R*obj.T);
       
     %pressure gradient force
     lift_force = dp * obj.A;
     
     %dynamical evolution of system
     xdot = [spike;                           %velocity
             lift_force/obj.M - obj.g;        %acceleration
             p_outside * p_inside;            %pressure change
             obj.solenoid_frequency*(u_thresh - x(4))];%solenoid response
           
    end
    
    function y = output(obj,~,x,u)
      % full state feedback is allowed here
      y = x;
    end

    
    function x_init = getInitialState(obj)
      %z_init = obj.ride_height + 1e-2;       
      z_init = obj.ride_height;
      %z_init = 1e-10;
      p_init = obj.x0(3);
      
      %p_init = obj.pa;      
      flow_init = obj.u0(1);
      
      x_init = [z_init;  %initial ride height
                0;       %initial z speed
                p_init;  %initial bottom pressure
                flow_init];   %input flow
    end
    
    function [c,V] = hoverLQR(obj, ROA)      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      %Q = diag([5 3 1e-4/obj.pressure_scale 5]);
      Q = diag([10 10 10 100]);
      R = 1e3;

      
      [A,B,C,D,xdot0] = obj.linearize(0,obj.x0,obj.u0);
      
      if ROA
        [c,V0] = tilqr(obj,x0,u0,Q,R);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        V=regionOfAttraction(pp,V0,options);
      else
        D = D(:, 1);
        Bw = B(:, 2:3);
        B = B(:, 1);
        
        [K,S] = lqr(full(A),full(B),Q,R);
        
        V.S = S;
        c.D = K;
        
        %[c, V0] = tilqr(obj,x0,u0,Q,R);
        %V = V0;
      end
    end
    
  end
  
end
