%m = 0.111;
m = 2.47e-3;
R = 0.02;
g = -9.81;
L = 1.0;
J = 2/3*m*R^2;

s = tf('s');
%P_ball = -m*g*d/L/(J/R^2+m)/s^2;
sys_c = ss(-m*g/(J/R^2+m)/s^2);
%sys_c.C = [sys_c.C; sys_c.C];


Ts = 0.01;
sys_d = c2d(sys_c, Ts, 'zoh');


num_input = 1;
num_output = 1;
num_state = 2;
num_disturbance = num_output;

Q = [5 0.1; 0.1 2];
R = 0.1;      
target_sys = sys_c;
%new state xdot
A = target_sys.A;
B1 = zeros(num_state, num_disturbance);
B2 = full(target_sys.B);


%fictitious output z      
C1 = [chol(Q); zeros(num_input, num_state)];
D11 = zeros(num_state+num_input, num_disturbance);
D12 = [zeros(num_state, num_input); R];

%output y
C2 = target_sys.C;      
D21 = eye(num_output);
%D22 = zeros(num_state, num_input);
D22 = target_sys.D;

%lump into augmented state space model
Ap = A;
Bp = [B1 B2];
Cp = [C1; C2];
Dp = [D11 D12; D21 D22];

P = ss(Ap, Bp, Cp, Dp);
%P = ss(Ap, Bp, Cp, Dp, Ts);
[K, CL, GAM, INFO] = hinfsyn(P, num_output, num_input);