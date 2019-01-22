p = AirSpringPlant1d_disturbed_2;


[A, B, C, D, xdot0] = p.linearize(0, p.x0, p.u0);

A = full(A);
B = full(B);
sys = ss(A, B, C, D, 'StateName', {'Height', 'Velocity', 'Bag Pressure', 'Solenoid Massflow'},...
                     'OutputName', {'Height', 'Velocity','Bag Pressure', 'Solenoid Massflow'},...
                     'InputName', {'MassFlowIn', 'W-TrackHeight', 'W-TrackGap'});

T5 = diag([1 1 1e-5 1]);
sys5 = ss2ss(sys, T5);
                 
u_sys = sys(:, 1);      %mass flow in   subsystem
wth_sys = sys(:, 2);    %w track height subsystem
wtg_sys = sys(:, 3);    %w track gap    subsystem

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
bode(wth_sys, opts)


subplot(1, 4, 2)
iopzmap(wth_sys)

t = linspace(0, tfinal, 1001);
subplot(1, 4, 3)
impulse(wth_sys, t)

subplot(1, 4, 4)
step(wth_sys, t)

figure(31)
subplot(1, 4, 1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
bode(wtg_sys, opts)


subplot(1, 4, 2)
iopzmap(wtg_sys)

t = linspace(0, tfinal, 1001);
subplot(1, 4, 3)
impulse(wtg_sys, t)

subplot(1, 4, 4)
step(wtg_sys, t)