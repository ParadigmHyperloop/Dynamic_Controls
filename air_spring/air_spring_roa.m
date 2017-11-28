p = AirSpringPlant1d;
%p = AirSpringPlant1d_disturbed;
x0 = Point(p.getStateFrame, p.x0);
u0 = Point(p.getInputFrame, p.u0);
pp = p.taylorApprox(0,x0,u0,3);  % make polynomial approximation

options = struct;
%options.method = {'levelset', 'bilinear'};
options.method = 'levelset';
%options.method = {'levelset', 'bilinear', 'sampling'}
options.degV = 2;
options.degL1 = 6;
options.degL2 = 6;
V = regionOfAttraction(pp, x0, options);

pref_inclusion = 'slice';
%pref_inclusion = 'projection';

figure(20);
subplot(1, 3, 1)
plotFunnel(V, struct('plotdims', [1 2], 'inclusion', pref_inclusion ))
xlabel('$$z \quad(m)$$', 'interpreter', 'latex')
ylabel('$$\dot{z} \quad(\frac{m}{s})$$', 'interpreter', 'latex')

subplot(1, 3, 2)
plotFunnel(V, struct('plotdims', [1 3], 'inclusion', pref_inclusion ))
xlabel('$$z \quad(m)$$', 'interpreter', 'latex')
ylabel('$$\Delta P \quad(Pa)$$', 'interpreter', 'latex')

subplot(1, 3, 3)
plotFunnel(V, struct('plotdims', [2 3], 'inclusion', pref_inclusion ))
xlabel('$$\dot{z} \quad(\frac{m}{s})$$', 'interpreter', 'latex')
ylabel('$$\Delta P \quad(Pa)$$', 'interpreter', 'latex')

%V0 = QuadraticLyapunovFunction(pp.getStateFrame, eye(3));
%
%options=struct();
%options.degL1=2;
%V=regionOfAttraction(pp,V0,options);

% %find a lyapunov function
% deg_V = 2;
% %fu = pp.getPolyDynamics;
% x = pp.getStateFrame.getPoly;
% 
% f_unscaled = pp.getPolyDynamics;
% f = subs(f_unscaled, x, x+p.x0);
% %open loop Lyapunov Function
% %f = subs(fu,pp.getInputFrame.getPoly, p.u0);
% %f = subs(subs(fu,pp.getInputFrame.getPoly, p.u0), x, x + p.x0);
% %f(3) = f(3) - subs(f(3), x, [0; 0; 0]); %get rid of floating point error
%                                   %we know that the origin has f(0) = 0;
% 
% prog = spotsosprog;
% prog = prog.withIndeterminate(x);
% [prog, V] = prog.newSOSPoly(monomials(x,2:deg_V));
% 
% Vdot = diff(V, x)*f;
% 
% prog = prog.withSOS(-Vdot);
% %prog = prog.withEqs(subs(V, x, p.x0)); %V(x0) = 0
% prog = prog.withEqs(subs(V, x, [0; 0; 0])); %V(0) = 0
% 
% %solver = @spot_mosek;
% solver = @spot_sedumi;
% options = spot_sdp_default_options();
% options.verbose = 1;
% 
% sol = prog.minimize(0, solver, options);
% 
% Vo = sol.eval(V)
% Vdoto = sol.eval(Vdot)
% 
% m1 = sol.gramMonomials{1};
% m2 = sol.gramMonomials{2};
% 
% S = double(sol.eval(sol.gramMatrices{1}));
% Sdot = double(sol.eval(sol.gramMatrices{2}));
% 
% %VLyap = QuadraticLyapunovFunction(p.getStateFrame, S);