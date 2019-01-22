classdef BallBeamDT < DrakeSystem
    properties
        %ping pong ball
        m = 2.47e-3; %mass [kg]
        R = 0.02; %radius [m]
        J_ball; %moment of inertia
        
        %environment
        g = -9.81; %gravity [m/s^2]
        
        %beam
        L = 0.1524; %beam length [m] (0.5')
        
        %control
        x0 = 0;
        u0 = 0;
        setpoint;
    end
   
    methods
        function obj = BallBeamDT(setpoint)
            % call the parent class constructor:
            obj = obj@DrakeSystem(...  
            0, ... % number of continuous states
            2, ... % number of discrete states
            1, ... % number of inputs
            2, ... % number of outputs
            false, ... % because the output does not depend on u
            true);  % because the update and output do not depend on t
            
            %rest of properties
            obj.setpoint = setpoint;
            obj.J_ball = 2/3*m*R^2;
            
            %coordinate frames
            obj = setInputFrame(obj, CoordinateFrame('BallBeamInput', 1, 'u', {'theta'}));
            obj = setStateFrame(obj, CoordinateFrame('BallBeamState', 2, 'x', {'r', 'rdot'}));
      
            obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback

        end
        
        function xnext = update(obj,t,x,u)
            %dynamics of system
            xnext = x^3;
        end
        
        function y=output(obj,t,x,u)
            y=x;
        end
  end
end
