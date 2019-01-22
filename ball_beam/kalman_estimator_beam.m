% 2.151: LQG regulator for coupled masses
% (See handout on discrete observers).
%---------------------------------------------------------
% Enter the continuous plant model
%
%m = 0.111;
m = 2.47e-3;
R = 0.02;
g = -9.81;
L = 0.1524;
J = 2/3*m*R^2;

s = tf('s');
%P_ball = -m*g*d/L/(J/R^2+m)/s^2;
sys_c = ss(-m*g/(J/R^2+m)/s^2);

A = sys_c.A;
B = sys_c.B;
C = sys_c.C;
D = sys_c.D;
%----------------------------------------------------------
% Create the LTI continuous model
%
%plant = ss(A, B, C, D, 'InputName', 'Force F (N)', 'OutputName', ...
%{'Displacement d (m)', 'Displacement y (m)', 'Force F (N)'});
%set(plant, 'StateName', {'d', 'd_dot', 'y', 'y_dot'})
%plant = ss(A, B, C, D, 'InputName', 'Angle', 'OutputName', {'r', 'rdot'});
plant = ss(A, B, C, D)
%----------------------------------------------------------
% Create the Discrete model with a sampling time of T=0.4 s.
%
T = 0.01;
ZOH = c2d(plant, T,'zoh');
% Retrieve the matrices
[Phi, Gamma, Cd, Dd] = ssdata(ZOH);
%----------------------------------------------------------
% Design the regulator by computing the LQR Gain matrix K
% Set R = [1] arbitrarily and scale the Q matrix accordingly.
% Make Q diagonal
%
R = 1;
%Q11 = input('LQR controller Q_11: ');
%Q22 = input('LQR controller Q_22: ');
%Q11 = 10;
%Q12 = 5;
%Q = diag([Q11 0 Q22 0]);
Q = [5 0.1; 1 0.1];
K = dlqr(Phi, Gamma, Q, R);
%----------------------------------------------------------
% Compute the Kalman filter gains
% Assume rms noise of 1% on each sensor channel
%Rv = 0.01^2 * eye(2);
Rv = 0.05^2;
% Input Rw
%Rw = input('Enter estimator Rw: ');
Rw = 0.001;
sensors = [1 1]; % d and y are sensed
known = [1]; % force u
P = ss(Phi, [Gamma Gamma], C, [D D], T);
[Observer, Ko] = kalman(P, Rw, Rv, [], sensors, known);
%-----------------------------------------------------------
% Create the regulator and the closed-loop system
%
lqg_reg = lqgreg(Observer, K, 'current');
feedin = [1]; % force u
feedout = [1]; % d and y
Gcl = feedback(ZOH, lqg_reg, feedin, feedout, +1);
%-----------------------------------------------------------
% Compute and plot the initial condition response
% Set x_1(0) = 1, all others to zero.
x0 = zeros(4,1);
x0(1) = 0.1;
figure(1), clf
initial(Gcl, x0)
% Plot all states and compare actual values with Kalman estimates
[y, t, x] = initial(Gcl, x0, 10);
figure(2), clf
subplot(2,1,1), stairs(t,x(:, [1 3])), grid, %legend('x_1=d', 'x_1hat', 0)
title('Response of states and predictive estimates to x_1(0) = 1'), ...
xlabel('Time (s)')
subplot(2,1,2), stairs(t,x(:, [2 4])), grid, %legend('x_2=ddot', 'x_2hat', 0)
xlabel('Time (s)')
%---------------------------------------------------------------
