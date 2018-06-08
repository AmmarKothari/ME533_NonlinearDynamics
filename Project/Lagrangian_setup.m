% setups things to use with project
% M(q)*q_dd + C(q,q_d)*q_d + K(q)*q + g(q) = Q
% Inertia Matrix
M = @(Q) Lagrangian_M(I1,I2,l1,l2,m1,m2,Q(2,1));

% Coriolis and Damping Matrix
% C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);
C = @(Q) Lagrangian_C(l1, l2, m2, Q(2,1), Q(1,2), Q(2,2));

% Spring
K = @() Lagrangian_K(k1,k2);

% Potential
G = @() Lagrangian_G(k1,k2,theta1_0,theta2_0);

% Generalized Forces
F = @(tau1,tau2,Q) Lagrangian_Q(b1,b2,tau1,tau2,Q(1,2), Q(2,2));

% Joint Angles
Q = zeros(m_inputs,3); % set initial conditions