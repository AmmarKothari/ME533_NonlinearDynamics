% setups things to use with project
% M(q)*q_dd + C(q,q_d)*q_d + K(q)*q + g(q) = Q
% Inertia Matrix
M = @(Q) Lagrangian_M(I1,I2,l1,l2,m1,m2,Q(2,1));
M2 = @(Q) [zeros(2,1), M(Q), zeros(2,2);
        zeros(2,1), zeros(2,2), M(Q)];

% Coriolis and Damping Matrix -- no damping in C, only in C_manual
% C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);
C = @(Q) Lagrangian_C(l1, l2, m2, Q(2,1), Q(1,2), Q(2,2));
C2 = @(Q) [zeros(2,1), C(Q(1), Q(2)), zeros(2,2);
        zeros(2,1), zeros(2,2), C(Q(3), Q(4))];

% Spring
KL = @() Lagrangian_K(k1,k2);
KR = @() Lagrangian_K(k1,k2);
K = @() [zeros(2,1), KR(), zeros(2,2);
        zeros(2,1), zeros(2,2), KL()];

% Potential
G = @() Lagrangian_G(k1,k2,theta1_0,theta2_0);

% Generalized Forces
F = @(tau1,tau2,Q) Lagrangian_Q(b1,b2,tau1,tau2,Q(1,2), Q(2,2));

% Joint Angles
Q = zeros(m_inputs,3); % set initial conditions        
        
% Coriolis and Damping Matrix
        

% Spring
K2 = @(Q) [zeros(2,1), K(Q(1), Q(2)), zeros(2,2);
        zeros(2,1), zeros(2,2), K(Q(3), Q(4))];


% Potential
G2 = @(Q) [0, G(Q(1), Q(2));
            0, G(Q(3), Q(4))];


