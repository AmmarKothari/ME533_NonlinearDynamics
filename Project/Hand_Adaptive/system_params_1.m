% System 1 Paremeters:

m_inputs = 5;
I_func = @(m, l) 1/12*m*l^2;

% % physical parameters of the hand
% Right
Rfing.m1 = 1; Rfing.l1 = 1;
Rfing.m2 = 2; Rfing.l2 = 1.2;

Rfing.I1 = I_func(Rfing.m1, Rfing.l1);
Rfing.I2 = I_func(Rfing.m2, Rfing.l2);

Rfing.k1 = 3.0; Rfing.k2 = 5.0;
Rfing.b1 = 4; Rfing.b2 = 3;
Rfing.theta1_0 = 0; % resting position of spring
Rfing.theta2_0 = 0;

% Left
Lfing = Rfing;

% system time constants
dt = 1e-3; % this needs to be atleast 1e-3, 1e-4 takes forever to finish!
t_end = 10;
ts = 0:dt:t_end;