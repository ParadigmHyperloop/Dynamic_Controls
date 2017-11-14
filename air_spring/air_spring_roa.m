p = AirSpringPlant1d;

pp = p.taylorApprox(0,Point(p.getStateFrame, p.x0),[],3);  % make polynomial approximation

%V0 = QuadraticLyapunovFunction(pp.getStateFrame, eye(3));
%
%options=struct();
%options.degL1=2;
%V=regionOfAttraction(pp,V0,options);

%find a lyapunov function
deg_V = 2;
%fu = pp.getPolyDynamics;
x = pp.getStateFrame.getPoly;

f_unscaled = pp.getPolyDynamics;
f = subs(f_unscaled, x, x+p.x0);
%open loop Lyapunov Function
%f = subs(fu,pp.getInputFrame.getPoly, p.u0);
%f = subs(subs(fu,pp.getInputFrame.getPoly, p.u0), x, x + p.x0);
%f(3) = f(3) - subs(f(3), x, [0; 0; 0]); %get rid of floating point error
                                  %we know that the origin has f(0) = 0;

prog = spotsosprog;
prog = prog.withIndeterminate(x);
[prog, V] = prog.newSOSPoly(monomials(x,2:deg_V));

Vdot = diff(V, x)*f;

prog = prog.withSOS(-Vdot);
%prog = prog.withEqs(subs(V, x, p.x0)); %V(x0) = 0
prog = prog.withEqs(subs(V, x, [0; 0; 0])); %V(0) = 0

solver = @spot_mosek;
options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(0, solver, options);

Vo = sol.eval(V)
Vdoto = sol.eval(Vdot)

S = double(sol.eval(sol.gramMatrices{1}));
Sdot = double(sol.eval(sol.gramMatrices{2}));

%VLyap = QuadraticLyapunovFunction(p.getStateFrame, S);