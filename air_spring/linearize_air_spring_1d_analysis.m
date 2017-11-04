p = AirSpringPlant1d;


[A, B, C, D, xdot0] = p.linearize(0, p.x0, p.u0);

A = full(A);
B = full(B);
sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Bag Pressure'},...
                     'OutputName', {'Height', 'Velocity','Bag Pressure'},...
                     'InputName', 'MassFlowIn');



tfinal = 0.2;
%tfinal = 15;

figure(29)
subplot(1, 4, 1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
bode(sys, opts)


subplot(1, 4, 2)
iopzmap(sys)

t = linspace(0, tfinal, 201);
subplot(1, 4, 3)
impulse(sys, t)

subplot(1, 4, 4)
step(sys, t)


