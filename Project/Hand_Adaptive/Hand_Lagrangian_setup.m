% setups things to use with project
% M(q)*q_dd + C(q,q_d)*q_d + K(q)*q + g(q) = Q
addpath('..')
% Inertia Matrix
MR = @(Q) Lagrangian_M_manual(Rfing.I1,Rfing.I2,Rfing.l1,Rfing.l2,Rfing.m1,Rfing.m2,Q(3,1));
ML = @(Q) Lagrangian_M_manual(Lfing.I1,Lfing.I2,Lfing.l1,Lfing.l2,Lfing.m1,Lfing.m2,Q(5,1));
M2 = @(Q) [zeros(2,1), MR(Q), zeros(2,2);
        zeros(2,1), zeros(2,2), ML(Q)];

% Coriolis and Damping Matrix -- no damping in C, only in C_manual
% C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);
CR = @(Q) Lagrangian_C_manual(Rfing.l1, Rfing.l2, Rfing.m2, Q(3,1), Q(2,2), Q(3,2));
CL = @(Q) Lagrangian_C_manual(Lfing.l1, Lfing.l2, Lfing.m2, Q(5,1), Q(4,2), Q(5,2));
C2 = @(Q) [zeros(2,1), CR(Q), zeros(2,2);
        zeros(2,1), zeros(2,2), CL(Q)];

% Spring
KR = @() Lagrangian_K(Rfing.k1,Rfing.k2);
KL = @() Lagrangian_K(Lfing.k1,Lfing.k2);
K = @() [zeros(2,1), KR(), zeros(2,2);
        zeros(2,1), zeros(2,2), KL()];

% Potential
GR = @() Lagrangian_G(Rfing.k1,Rfing.k2,Rfing.theta1_0,Rfing.theta2_0);
GL = @() Lagrangian_G(Lfing.k1,Lfing.k2,Lfing.theta1_0,Lfing.theta2_0);
G = @() [zeros(2,1), GR(), zeros(2,2);
        zeros(2,1), zeros(2,2), GL()];

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


