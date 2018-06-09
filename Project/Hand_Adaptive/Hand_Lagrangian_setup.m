% setups things to use with project
% M(q)*q_dd + C(q,q_d)*q_d + K(q)*q + g(q) = Q
addpath('..')
% function to turn the individual finger matrices into a single system
% M1, M2 should be 2x2
CreateDiagnolishMatrix = @(M1, M2) [zeros(1,5);
                                    zeros(2,1), M1, zeros(2,2);
                                    zeros(2,1), zeros(2,2), M2];

% Inertia Matrix
MR = @(Q) Lagrangian_M_manual(Rfing.I1,Rfing.m1,Rfing.l1,...
                            Rfing.I2,Rfing.m2,Rfing.l2,...
                            Q(2,1), Q(3,1));
ML = @(Q) Lagrangian_M_manual(Lfing.I1,Lfing.m1,Lfing.l1,...
                            Lfing.I2,Lfing.m2,Lfing.l2,...
                            Q(4,1), Q(5,1));
M2 = @(Q) CreateDiagnolishMatrix(MR(Q), ML(Q));
% M2 = @(Q) [zeros(2,1), MR(Q), zeros(2,2);
%         zeros(2,1), zeros(2,2), ML(Q)];

% Coriolis and Damping Matrix -- no damping in C, only in C_manual
% C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);
CR = @(Q) Lagrangian_C_manual(Rfing.I1,Rfing.m1,Rfing.l1,...
                            Rfing.I2,Rfing.m2,Rfing.l2,...
                            Q(2,1), Q(3,1), ...
                            Q(2,2), Q(3,2), ...
                            Rfing.b1, Rfing.b2);
CL = @(Q) Lagrangian_C_manual(Lfing.I1,Lfing.m1,Lfing.l1,...
                            Lfing.I2,Lfing.m2,Lfing.l2,...
                            Q(4,1), Q(5,1), ...
                            Q(4,2), Q(5,2), ...
                            Lfing.b1, Lfing.b2);
C2 = @(Q) CreateDiagnolishMatrix(CR(Q), CL(Q));
% C2 = @(Q) [zeros(2,1), CR(Q), zeros(2,2);
%         zeros(2,1), zeros(2,2), CL(Q)];

% Spring
KR = @() Lagrangian_K_manual(Rfing.k1,Rfing.k2);
KL = @() Lagrangian_K_manual(Lfing.k1,Lfing.k2);
K2 = @() CreateDiagnolishMatrix(KR(), KL());
% K2 = @() [zeros(2,1), KR(), zeros(2,2);
%         zeros(2,1), zeros(2,2), KL()];

% % Potential
GR = @() Lagrangian_G(Rfing.k1,Rfing.k2,Rfing.theta1_0,Rfing.theta2_0);
GL = @() Lagrangian_G(Lfing.k1,Lfing.k2,Lfing.theta1_0,Lfing.theta2_0);
G2 = @() [0;
        GR();
        GL()];

% Generalized Forces
% F = @(tau1,tau2,Q) Lagrangian_Q(b1,b2,tau1,tau2,Q(1,2), Q(2,2));
FR = @(tau1, tau2) [tau1; tau2];
FL = @(tau3, tau4) [tau3; tau4];
F2 = @(Tau) [Tau(1);
            FR(Tau(2), Tau(3));
            FL(Tau(4), Tau(5))];


