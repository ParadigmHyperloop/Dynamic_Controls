% 2.151: LQG regulator for coupled masses
% (See handout on discrete observers).
%---------------------------------------------------------
% Enter the continuous plant model
%
M = 1; m = 0.1; k = 0.091; b = 0.0036;
A = [ 0 1 0 0
-k/m -b/m k/m b/m
0 0 0 1
k/M b/M -k/M -b/M];
B = [0; 0; 0; 1/M];
C = [1 0 0 0
0 0 1 0
0 0 0 0]; % get d, y and u as outputs
D = [0; 0; 1];
%----------------------------------------------------------
% Create the LTI continuous model
%
plant = ss(A, B, C, D, 'InputName', 'Force F (N)', 'OutputName', ...
{'Displacement d (m)', 'Displacement y (m)', 'Force F (N)'});
set(plant, 'StateName', {'d', 'd_dot', 'y', 'y_dot'})
%----------------------------------------------------------
% Create the Discrete model with a sampling time of T=0.4 s.
%
T = 0.4;
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
Q11 = 10;
Q12 = 5;
Q = diag([Q11 0 Q22 0]);
K = dlqr(Phi, Gamma, Q, R);
%----------------------------------------------------------
% Compute the Kalman filter gains
% Assume rms noise of 1% on each sensor channel
Rv = 0.01^2 * eye(2);
% Input Rw
%Rw = input('Enter estimator Rw: ');
Rw = 0.001;
sensors = [1, 2]; % d and y are sensed
known = [1]; % force u
P = ss(Phi, [Gamma Gamma], C, [D D], T);
[Observer, Ko] = kalman(P, Rw, Rv, [], sensors, known);
%-----------------------------------------------------------
% Create the regulator and the closed-loop system
%
lqg_reg = lqgreg(Observer, K, 'current');
feedin = [1]; % force u
feedout = [1, 2]; % d and y
Gcl = feedback(ZOH, lqg_reg, feedin, feedout, +1);
%-----------------------------------------------------------
% Compute and plot the initial condition response
% Set x_1(0) = 1, all others to zero.
x0 = zeros(8,1);
x0(1) = 1;
figure(1), clf
initial(Gcl, x0)
% Plot all states and compare actual values with Kalman estimates
[y, t, x] = initial(Gcl, x0, 10);
figure(2), clf
subplot(2,2,1), stairs(t,x(:, [1 5])), grid, %legend('x_1=d', 'x_1hat', 0)
title('Response of states and predictive estimates to x_1(0) = 1'), ...
xlabel('Time (s)')
subplot(2,2,2), stairs(t,x(:, [2 6])), grid, %legend('x_2=ddot', 'x_2hat', 0)
xlabel('Time (s)')
subplot(2,2,3), stairs(t,x(:, [3 7])), grid, %legend('x_3=y', 'x_3hat', 0)
xlabel('Time (s)')
subplot(2,2,4), stairs(t,x(:, [4 8])), grid, %legend('x_4=ydot', 'x_4hat', 0)
xlabel('Time (s)')
%---------------------------------------------------------------
