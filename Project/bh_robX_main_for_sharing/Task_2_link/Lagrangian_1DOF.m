% Deriving the equations of motion for a 1 DOF planar rotating rod

I_func = @(m, l) 1/3*m*l^2;

% Model Parameters
syms m1 k1 b1 tau1

% geometry Parameters
syms l1 theta1(t) t theta1_0
% syms theta1_d(t)

I1 = I_func(m1, l1);

% COM Position of link in cartesian space
X1_COM = (l1/2)*cos(theta1);
Y1_COM = (l1/2)*sin(theta1);

% COM Velocity of link in cartesian space
X1_COM_d = diff(X1_COM, t);
Y1_COM_d = diff(Y1_COM, t);

% COM Acceleration of link in cartesian space
X1_COM_dd = diff(X1_COM, t, 2);
Y1_COM_dd = diff(Y1_COM, t, 2);

% Joint Velocity
theta1_d = diff(theta1, t);

% Joint Acceleration
% theta1_dd = diff(theta1, t, 2);

% Define Kinetic Energy of System
% % Why isn't this v which is equal to sqrt(x_d^2 + y_d^2)
KE_trans = 0.5*m1*(X1_COM_d^2 + Y1_COM_d^2);
KE_rot = 0.5*I1*theta1_d^2;
KE = KE_trans + KE_rot;

% Define Potential Energy of System
% % no gravity, but potential energy of spring
PE = 0.5*k1*(theta1(t) - theta1_0);

% Define Lagrangian
L = KE - PE;

% take derivative
% Need to add this package from file exchange
% Euler-Lagrange tool package
syms th1 th1_d
L_sub_d = subs(L, theta1_d, th1_d);
dl_dqd = diff(L_sub_d, th1_d);
d_dt_dl_dqd = diff(dl_dqd, t);
L_sub = subs(L, theta1, th1);
dl_dq = diff(L_sub, th1);
eq_old = d_dt_dl_dqd - dl_dq;
eq = subs(eq_old, th1, theta1); 
eq = subs(eq, th1_d, theta1_d);

% X = {theta1(t), theta1_d(t)};
% Q_i = {0}; Q_e = {0}; % No generalized forces 
% R = b1*theta1_d^2/2; % Friction term 
% par = {m1 l1 k1 b1}; % System parameters 
% VF = EulerLagrange(L,X,Q_i,Q_e,R,par,'m'); 