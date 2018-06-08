m_inputs = 5;
I_func = @(m, l) 1/12*m*l^2;

% physical parameters of the arm
m1 = 1;
m2 = 2;
I1 = I_func(m1, l1);
I2 = I_func(m2, l2);

lc1 = l1/2;
lc2 = l2/2;

k1 = 0.0; k2 = 1.0;
b1 = 0; b2 = 5;
theta1_0 = 0; % resting position of spring
theta2_0 = 0;

% system time constants
dt = 1e-3; % this needs to be atleast 1e-3, 1e-4 takes forever to finish!
t_end = 10;
ts = 0:dt:t_end;