classdef SpringRocketPlant < SecondOrderSystem

  % state (world coordinates):  
  %  q(1) - bar x position 
  %  q(2) - bar z position
  %  q(3) - pitch (theta)
  %  q(4) - left  engine x position 
  %  q(5) - left  engine z position 
  %  q(6) - right engine x position 
  %  q(7) - right engine z position 
  
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    %number
    Nx = 7; %number of states
    Nu = 2; %number of inputs
      
    %bar rocket properties
    L = 0.25;       % length of rotor arm
    mb = 0.486;      % mass of quadrotor
    I = 0.00383;    % moment of inertia
    
    %left engine properties
    m1 = 0.1;
    k1 = 1;
    c1 = 0.6;
    l1 = 0;
    
    %right engine properties
    m2 = 0.1;
    k2 = 1;
    c2 = 0.6;
    l2 = 0;
    
    %environment properties
    g = 9.81;       % gravity      
    b = 1;
    
    x0;
    u0;
    M;
    
  end
  
  methods
    function obj = SpringRocketPlant()
      obj = obj@SecondOrderSystem(7,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    
      %obj.umin = [0; 0];
      obj.M = obj.mb + obj.m1 + obj.m2;
      [obj.x0, obj.u0] = obj.get_equilibrium();
    end
    
    function [x0, u0] = get_equilibrium(obj)
        %equilibrium position
        x0_pos = [0;
              0;
              0;
              -obj.L/2;
              -obj.l1;
              -obj.L/2;
              -obj.l2];
        x0 = [x0_pos; zeros(obj.Nx, 1)];
        u0 = [1; 1] * obj.M*obj.g / 2;
    end
    
    function qdd = sodynamics(obj,~,q,qd,u)
      %Implement the second-order dynamics for the bar rocket
      %time invariant system
      %f(x) + g(x) u
      %nonlinearity affine in control when the spring natural lengths are
      %zero. Otherwise, nonlinearities are all over the place.
            
      %force term
      mass_inertia = [obj.mb; 
                      obj.mb; 
                      obj.I;
                      obj.m1;
                      obj.m1;
                      obj.m2;
                      obj.m2];

      %body coordinates instead of world coordinates      
      
      %distances 
      dx1 = q(4) - q(1) + obj.L/2;
      dx2 = q(6) - q(1) - obj.L/2;
      dz1 = q(5) - q(2);
      dz2 = q(7) - q(2);
      
      %radial stretching of springs
      dr1 = sqrt(dx1^2 + dz1^2);
      dr2 = sqrt(dx2^2 + dz2^2);
      %dr1 = hypot(dx1, dz1);
      %dr2 = hypot(dx2, dz2);
      
      %relevant trig functions of angles
      sin_a = dx1/dr1;
      cos_a = dz1/dr1;
      sin_b = dx2/dr2;
      cos_b = dz2/dr2;
            
      %velocity differences
      dxd1 = qd(4) - qd(1);
      dxd2 = qd(6) - qd(1);
      dzd1 = qd(5) - qd(2);
      dzd2 = qd(7) - qd(2);
      
      
      %state forces and evolution
      
      %friction and velocity damping
      friction = -obj.b*[1; 1; 0; 1; 1; 1; 1].*qd;      
      
      %gravitational effects              
      gravity = [-obj.mb*obj.g*sin(q(3));
                 -obj.mb*obj.g*cos(q(3));
                 0;
                 -obj.m1*obj.g*sin(q(3));
                 -obj.m1*obj.g*cos(q(3));
                 -obj.m2*obj.g*sin(q(3));
                 -obj.m2*obj.g*cos(q(3))];
      
      %spring force is k*(x1 - x - natural length)
      %hooke's law for elasticity
      spring_force = [obj.k1*(dx1) + obj.k2*(dx2);
                      obj.k1*(dz1) + obj.k2*(dz1);
                      obj.k1*(dz1) - obj.k2*(dz2);
                      
                      obj.k1*(-dx1);
                      obj.k1*(-dz1);
                      obj.k2*(-dx2);
                      obj.k2*(-dx2)];
      
      %spring damping force            
      damp_force   = [obj.c1*(dxd1) + obj.c2*(dxd2);
                      obj.c1*(dzd1) + obj.c2*(dzd1);
                      obj.c1*(dzd1) - obj.c2*(dzd2);
                      
                      obj.c1*(-dxd1);
                      obj.c1*(-dzd1);
                      obj.c2*(-dxd2);
                      obj.c2*(-dxd2)];
      
                  
      f = friction + gravity + spring_force + damp_force;
      
      
                  
      %response to input force
      force_response = [ 0,     0;
                         0,     0;
                         0,     0;
                         sin_a, 0;
                         cos_a, 0;
                         0,     sin_b;
                         0,     cos_b];
      gu = force_response*u;    
      
      
      
      qdd = (f + gu) ./ mass_inertia;
    end
    
    function x = getInitialState(obj)
      x = randn(14,1);
      %x = [0.8; -0.5; 0.5; 0; 0; 0];
    end
    
    function [c,V] = hoverLQR(obj)
      x0 = Point(obj.getStateFrame, obj.x0);
      u0 = Point(obj.getInputFrame, obj.u0);
      
      Q_diag = ones(obj.Nx*2, 1);
      Q_diag(obj.Nx+3) = obj.L/2/pi;
      Q_diag(1:obj.Nx) = Q_diag(1:obj.Nx)*10;
      
      Q = diag(Q_diag);
      
      R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

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
