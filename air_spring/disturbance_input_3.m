classdef disturbance_input_3 < DrakeSystem

%bag is in full contact with the ground
%includes pod traveling at constant velocity   
  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - solenoid response [single pole filter]
  % input:
  %  u(1) - DISTURBANCE (change in track height)
  %  u(2) - DISTURBANCE (track slot width)

  properties  
      u0; %nominal input
      v;  %pod velocity in m/s
      %slot_length  = 0.1;
      %slot_length  = 0.01;      
      slot_length  = 0.00635; %0.25" [m]
      track_length = 3.81;    %12.5' [m]
      %track_length  = 1.8288;  %6'    [m]
      %square wave of height
      
      %track drift
      track_drift_max = 0.01016; %0.4" (need to check these numbers)
      track_drift_incremental = 0.001016; %0.04"

      
      track_period;
      track_duty_cycle;
      
      track_drift = 0.001016; %0.04"
      
      %time between slab changes
      sample_time;
      seed;
  end
  
  methods
    function obj = disturbance_input_3(p, v, seed)
      obj = obj@DrakeSystem(0, ... %number of continuous states
                                  0, ... %number of discrete states
                                  0, ... %number of inputs
                                  2, ... %number of output
                                  true, ... %because the output does not depend on u
                                  false); %because the dynamics and output do not depend on t      
      obj.u0 = p.u0;
      obj.v = v;
      
      obj.track_period = obj.track_length + obj.slot_length;
      obj.track_duty_cycle = 100*(obj.track_length / obj.track_period);
      
      obj.sample_time = obj.track_period / v;
      obj.seed = seed;
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
    function y = output(obj,t, ~,~)
      % full state feedback is allowed here
      %u_base = obj.u0(1);
      wth_base = 0;
      
      x = obj.v*t;
      ws = square(2*pi.*x/obj.track_period, obj.track_duty_cycle);
%     ws = 1;
      wtg_base = -(ws/2 - 0.5)*obj.slot_length;
      %wtg_base = 0;
      
      %wth_base = 0;
      
      num_calls = floor(x/obj.track_period);
      wth_base = obj.step_rand_noise(obj.track_drift_incremental, num_calls);
      
      
      y = [wth_base; wtg_base];

    end  
    
    function s = step_rand_noise(obj, drift, num_calls)
        %integrated uniform random noise process
        %truly horrible rng hacking
        rng(obj.seed);
        steps = (rand(1, num_calls)-0.5) * 2 * drift;
        s = sum(steps);
    end
    
  end
  
end
