classdef BallBeamSimple < DrakeSystem
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
        
        %sample time
        Ts = 0.01; %10ms
    end
   
    methods
        function obj = BallBeamSimple(setpoint)
            % call the parent class constructor:
            obj = obj@DrakeSystem(...  
            2, ... % number of continuous states
            0, ... % number of discrete states
            1, ... % number of inputs
            2, ... % number of outputs
            false, ... % because the output does not depend on u
            true);  % because the update and output do not depend on t
            
            %rest of properties
            obj.setpoint = setpoint;
            obj.J_ball = 2/3*obj.m*obj.R^2;
            
            %coordinate frames
            obj = setInputFrame(obj, CoordinateFrame('BallBeamInput', 1, 'u', {'theta'}));
            obj = setStateFrame(obj, CoordinateFrame('BallBeamState', 2, 'x', {'r', 'rdot'}));
      
            obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback

            obj.x0 = [setpoint; 0];
            obj.u0 = obj.m*obj.g*setpoint;
            
            %discrete output
            %obj.setSampleTime([obj.Ts 0; obj.Ts 0]);
        end
        
%         function xnext = update(obj,t,x,u)
%             %dynamics of system
%             xnext = x^3;
%         end
        function xdot = dynamics(obj, t, x, u)
            xd1 = x(2);
            xd2 = -obj.m*obj.g*sin(u)/(obj.m + obj.J_ball/obj.R^2);
            
            xdot = [xd1; xd2];
        end
        
        function y=output(obj,t,x,u)
            y=x;
        end
        
        function x0 = getInitialState(obj)
            x0 = [0; 0];
        end
        
        function [c] = balanceLQRD(obj)      
          Q = [5 0.1; 0.1 1];
          R = 1;

          %c = tilqr(obj,x0,u0,Q,R);          
          [A, B, C, D] = obj.linearize(0, obj.x0, obj.u0);
          
          [K, S] = lqrd(full(A), full(B), Q, R, obj.Ts);
          
          c = LinearSystem([],[],[],[],[],-K);
          c = c.setInputFrame(obj.getOutputFrame);
          c = c.setOutputFrame(obj.getInputFrame);
        end

  end
end
