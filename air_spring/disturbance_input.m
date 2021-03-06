classdef disturbance_input < DrakeSystem

%bag is in full contact with the ground
%purely heave motion, no horizontal travel
    
  % state (world coordinates):  
  %  x(1) - z position [cm]
  %  x(2) - z speed    [cm/s]
  %  x(3) - bag pressure   [N/m^2 = Pa]
  %  x(4) - solenoid response [single pole filter]
  % input:
  %  u(1) - mass flow into bag
  %  u(2) - DISTURBANCE (change in track height)

  properties  
      u0; %nominal input
  end
  
  methods
    function obj = disturbance_input(p)
      obj = obj@DrakeSystem(0, ... %number of continuous states
                                  0, ... %number of discrete states
                                  4, ... %number of inputs
                                  2, ... %number of output
                                  true, ... %because the output does not depend on u
                                  false); %because the dynamics and output do not depend on t      
      %obj = obj.setOutputFrame(p.getInputFrame);  % allow full-state feedback
      obj.u0 = p.u0;

    end
            
    function y = output(obj,t,x,u)
      % full state feedback is allowed here
      u_base = obj.u0(1);
      if (t < 0.5) || (t > 0.51)
          w_base = 0;
      else
          %w_base = -5e-3;
          w_base = -0.0127;
      end
      y = [u_base; w_base];
      %y = obj.u0;
    end  
    
  end
  
end
