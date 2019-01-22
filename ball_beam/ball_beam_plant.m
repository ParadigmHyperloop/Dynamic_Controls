classdef ball_beam_plant < DrakeSystem

  % state (canonical coordinates):  
  %  q(1) - position of ball along beam [m]
  %  q(2) - angle of beam [rad]
  % input:
  %  u(1) - motor torque

  %equations from http://www.laccei.org/LACCEI2014-Guayaquil/RefereedPapers/RP176.pdf
  
  properties
    %beam properties
    L = 0.2;        % length of beam [m]
    m_beam = 0.5;   % mass of beam [kg]
    J_beam;         % beam inertia [kg m^2]
    
    %ball properties
    r_ball = 0.02;  % radius of ball [m]
    m_ball = 0.01;  % mass of ball [kg]
    J_ball;         % ball inertia [kg m^2]
    
    %environment properties
    g = 9.81;       % gravity  
    
    %controls
    x0;             %equilibrium point (setpoint)
    u0;             %equilibrium control input (torque)
    uses_observer = 0;
  end
  
  methods
    function obj = ball_beam_plant(setpoint)
      %obj = obj@SecondOrderSystem(2,1,true);
      obj = obj@DrakeSystem(4, ... %number of continuous states
                            0, ... %number of discrete states
                            1, ... %number of inputs
                            4, ... %number of output
                            false, ... %because the output does not depend on u
                            true); %because the dynamics and output do not depend on t      
      
      %coordinate frames     
      %obj = setInputFrame(obj,CoordinateFrame('TWIPInput',1,'u',{'tau'}));
      
      obj = setInputFrame(obj, CoordinateFrame('BallBeamInput', 1, 'u', {'tau'}));
      obj = setStateFrame(obj, CoordinateFrame('BallBeamState', 4, 'x', {'r', 'rdot', 'th', 'thdot'}));
      
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
      
      %moments of inertia
      obj.J_beam = obj.m_beam*obj.L^2/12;
      obj.J_ball = 2*obj.m_ball*obj.r_ball^2 / 3; %hollow sphere
      
      %IMPORTANT!
      if nargin > 0
          ball_setpoint = setpoint;
      else
          ball_setpoint = 0.0; %r0;
      end
      %ball_setpoint = 0.0; %r0;
      
      
      %equilibrium point
      obj.x0 = [ball_setpoint; 0; 0; 0];
      obj.u0 = obj.m_ball*obj.g*ball_setpoint;       
      
    end
    
    %function y = output(obj,t, w,~)
    function y = output(obj, t, x, u)
        y = x;
    end
    
    function xdot = dynamics(obj,t,x,u)
      %Implement the second-order dynamics for the ball_beam system
      
      %q(1) position of ball along beam, qd(1) velocity (r)
      %q(2) angle of beam, qd(2) angular velocity       (th)
      %u motor torque required
      
      %open loop control
      %torque = 0;
      
      %closed loop
      torque = u;
      
      %mass/inertia matrix
      M = [obj.J_ball/obj.L^2 + obj.m_ball; obj.m_ball*x(1)^2 + obj.J_beam];
      
      rdd  = obj.m_ball*(x(1)*x(4)^2 - obj.g*sin(x(3)));
      thdd = -2*obj.m_ball*x(1)*x(2)*x(4) - obj.m_ball*obj.g*x(1)*cos(x(3)) + torque; 
      
      xdot = [x(2); rdd/M(1); x(4); thdd/M(2)];
    
    end
    
    function x = getInitialState(obj)
      x = [0.1; 0; 0; 0];
    
    end
    
    function [c,V] = balanceLQR(obj)
      
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      
      Qr = [5 0.1; 0.1 1];
      Qth = [1 0.01; 0.01 0.1];
      Q = blkdiag(Qr, Qth);
      %Q = diag([10; 10; 1; (obj.L/2/pi)]);
      %Q = eye(4);
      R = 30;
      
      if (nargout>1)
        [c,V0] = tilqr(obj,x0,u0,Q,R);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        V=regionOfAttraction(pp,V0,options);
      else
        c = tilqr(obj,x0,u0,Q,R);
      end
    end
    
  end
  
end
