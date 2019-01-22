%m = 0.111;
m = 2.47e-3;
R = 0.02;
g = -9.81;
L = 1.0;
J = 2/3*m*R^2;

s = tf('s');
%P_ball = -m*g*d/L/(J/R^2+m)/s^2;
sys_c = ss(-m*g/(J/R^2+m)/s^2);

Ts = 1/100;
ball_d = c2d(sys_c,Ts,'zoh');
sys_d = ss(ball_d);

%rlocus(ball_d)
%bodeplot(sys_c,'b',sys_d,'r')

Q = [5 0.1; 0.1 1];
R = 1;

[K, S] = lqrd(sys_d.A, sys_d.B, Q, R, Ts);

sys_dcl = sys_d;
sys_dcl.A = sys_d.A-sys_d.B*K;

%observer placement
P = [-0.2 -0.21];
L = place(sys_d.A', sys_d.C', P)';


% %H infinity stuff
% num_state = 2;
% num_disturbance = 1;
% num_input = 1;
% num_output = 1;
% 
% %new state xdot
% A = full(sys_d.A);
% B1 = zeros(num_state, num_disturbance);
% B2 = full(sys_d.B);
% 
% 
% %fictitious output z      
% C1 = [chol(Q); zeros(num_input, num_state)];
% D11 = zeros(num_state+num_input, num_disturbance);
% D12 = [zeros(num_state, num_input); R];
% 
% %output y
% C2 = sys_d.C;      
% D21 = eye(num_output);
% %D22 = zeros(num_state, num_input);
% D22 = sys_d.D;
% 
% %lump into augmented state space model
% Ap = A;
% Bp = [B1 B2];
% Cp = [C1; C2];
% Dp = [D11 D12; D21 D22];
% 
% P = ss(Ap, Bp, Cp, Dp);
% 
% %synthesize H infinity controller
% [K, CL, GAM, INFO] = hinfsyn(P, num_input, num_output);
