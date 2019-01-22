%m = 0.111;
m = 2.47e-3;
R = 0.02;
g = -9.81;
L = 1.0;
J = 2/3*m*R^2;

s = tf('s');
%P_ball = -m*g*d/L/(J/R^2+m)/s^2;
sys_c = ss(-m*g/(J/R^2+m)/s^2);
sys_c.C = [sys_c.C; sys_c.C];
% sys_c.D = [[sys_c.D]; [sys_c.D]];

Ts = 0.01;
sys_d = c2d(sys_c, Ts, 'zoh');

R = 1;
Q = [4 2; 2 1];
QXU = blkdiag(Q, R);

w_mag = 0.01;
v_mag = 0.12;
%QXV = diag([w_mag; w_mag; v_mag]);
QXV = diag([w_mag; w_mag; v_mag; v_mag]);
QI = eye(2);

reg = lqg(sys_d, QXU, QXV, QI, '1dof')