classdef disturbance_input_2 < DrakeSystem

%bag is in full contact with the ground
%includes pod traveling at constant velocity   
  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - solenoid response [single pole filter]
  % input:
  %  u(1) - mass flow into bag
  %  u(2) - DISTURBANCE (change in track height)
  %  u(3) - DISTURBANCE (track slot width)

  properties  
      u0; %nominal input
      v;  %pod velocity in m/s
      %slot_length  = 0.1;
      %slot_length  = 0.01;      
      slot_length  = 0.00635; %0.25" [m]
      %track_length = 3.81;    %12.5' [m]
      track_length  = 1.8288;  %6'    [m]
      %square wave of height
      track_period;
      track_duty_cycle;
      
      track_drift = 0.001016; %0.04"
      
      %time between slab changes
      sample_time;
  end
  
  methods
    function obj = disturbance_input_2(p, v)
      obj = obj@DrakeSystem(0, ... %number of continuous states
                                  0, ... %number of discrete states
                                  4, ... %number of inputs
                                  3, ... %number of output
                                  true, ... %because the output does not depend on u
                                  false); %because the dynamics and output do not depend on t      
      obj.u0 = p.u0;
      obj.v = v;
      
      obj.track_period = obj.track_length + obj.slot_length;
      obj.track_duty_cycle = 100*(obj.track_length / obj.track_period);
      
      obj.sample_time = obj.track_period/v;
    end
%     
%     function w_init = getInitialState(obj)
%         w_init = 0;
%     end
% 
%     
%     function ts = getSampleTime(obj)
%         %ts = [obj.sample_time; 0];
%         ts = [1; 0];
%     end
%     
%     function w_next = update(obj, t, w, ~)
%         new_rand = 2*(rand-0.5);
%         track_change = obj.track_drift * new_rand;
%                 
%         w_next = w + track_change;
%         %w_next = w;
%     end
%             
    function y = output(obj,t, w,~)
      % full state feedback is allowed here
      u_base = obj.u0(1);
      wth_base = 0;
%       if (t < 0.5) || (t > 0.6)
%           wtg_base = 0;
%       else
%           %wth_base = -5e-3;
%           %wth_base = -0.0127;
%           wtg_base = 0.00635;
%       end

      x = obj.v*t;
      ws = square(2*pi.*x/obj.track_period, obj.track_duty_cycle);
%     ws = 1;
      wtg_base = -(ws/2 - 0.5)*obj.slot_length;
      
      
%       if wtg_base ~= 0          
%         new_rand = 2*(rand-0.5);
%         track_change = obj.track_drift * new_rand;
%         wth_base = track_change;
%       end
      %wth_base = -5e-3;
      wth_base = 0;
      %wth_base = w;

      y = [u_base; wth_base; wtg_base];

    end  
    
  end
  
end
