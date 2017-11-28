p = AirSpringPlant1d_disturbed;


[A, B, C, D, xdot0] = p.linearize(0, p.x0, p.u0);

A = full(A);
B = full(B);
sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Bag Pressure', 'Solenoid Massflow'},...
                     'OutputName', {'Height', 'Velocity','Bag Pressure', 'Solenoid Massflow'},...
                     'InputName', {'MassFlowIn', 'W-TrackHeight'});

u_sys = sys(:, 1);
w_sys = sys(:, 2);
                 
%tfinal = 0.25;
tfinal = 1;

figure(29)
subplot(1, 4, 1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
bode(u_sys, opts)


subplot(1, 4, 2)
iopzmap(u_sys)

t = linspace(0, tfinal, 1001);
subplot(1, 4, 3)
impulse(u_sys, t)

subplot(1, 4, 4)
step(u_sys, t)

figure(30)
subplot(1, 4, 1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
bode(w_sys, opts)


subplot(1, 4, 2)
iopzmap(w_sys)

t = linspace(0, tfinal, 1001);
subplot(1, 4, 3)
impulse(w_sys, t)

subplot(1, 4, 4)
step(w_sys, t)



