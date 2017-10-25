classdef nonlinear_example < DrakeSystem
  properties
    x0 = 1;
    u0 = 0;
    delta = 0.1;
  end
  
  methods
    function obj = nonlinear_example()
      % call the parent class constructor:
      obj = obj@DrakeSystem(...  
         1, ... % number of continuous states
         0, ... % number of discrete states
         1, ... % number of inputs
         1, ... % number of outputs
         false, ... % because the output does not depend on u
         true);  % because the dynamics and output do not depend on t
     
     obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    end
    
    function x = getInitialState(obj)
      x = 1.5; 
    end
    
    function xdot = dynamics(obj,t,x,u)
      %xdot = -x+x^3 + u;
     % xdot = 1 - sqrt(x) + u;
      xdot = 1 - power(x, 1/3.0) + u;
    end
    
    
    function y=output(obj,t,x,u)
      y=x;
    end
    
    function [c,V] = hoverLQR(obj)      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      Q = diag([1]);
      R = diag(1)*obj.delta;

      [c, V0] = tilqr(obj,x0,u0,Q,R);
      V = V0;
    end

  end
end
