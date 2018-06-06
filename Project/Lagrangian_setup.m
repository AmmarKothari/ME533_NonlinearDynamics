% setups things to use with project
% M(q)*q_dd + C(q,q_d)*q_d + K(q)*q + g(q) = Q
% Inertia Matrix
M = @(q1, q2) Lagrangian_M(I1,I2,l1,l2,m1,m2,q2);

% Coriolis and Damping Matrix
C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);

% Spring
K = @() Lagrangian_K(k1,k2);

% Potential
G = @() Lagrangian_G(k1,k2,theta1_0,theta2_0);

% Generalized Forces
F = @(tau1,tau2,q1_d,q2_d) Lagrangian_Q(b1,b2,tau1,tau2,q1_d,q2_d);

% Joint Angles
q1 = 0; q2 = 0;
q1_d = 0; q2_d = 0;

Q = [q1; q2];
Q_d = [q1_d; q2_d];