% system setup using lagrangian equations that have been derived

% setting up system parameters
system_params_1();

% setting up functions
Lagrangian_setup();

% target Joint angle -- for position tracking
% Constant trajectory
% q1_target_func = @(t) 0.9;
% q2_target_func = @(t) 0.95;
% Varying trajectory
q1_target_func = @(t) pi/6*(1-cos(2*pi*t));
q2_target_func = @(t) pi/4*(1-cos(2*pi*t));

% q1_target = q1_target_func(ts');
% q2_target = q2_target_func(ts');

q1_target = [q1_target_func(ts(1:100)'); q1_target_func(ts(100))*ones(length(ts(101:end)),1)];
q2_target = [q2_target_func(ts(1:100)'); q2_target_func(ts(100))*ones(length(ts(101:end)),1)];

q1_target_d = [0; diff(q1_target)];
q1_target_dd = [0; diff(q1_target_d)];

q2_target_d = [0; diff(q2_target)];
q2_target_dd = [0; diff(q2_target_d)];

Q_target = @(i_t) [q1_target(i_t), q1_target_d(i_t), q1_target_dd(i_t);
                   q2_target(i_t), q2_target_d(i_t), q2_target_dd(i_t)];
% Q_target_d = @(i_t) [; ];
% Q_target_dd = @(i_t) [; ];

% Control law
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %errors [position, velocity, acceleration]
% ed = @(Q_d, Q_target_d) Q_d - Q_target_d; % velocity error
torque_limit = 1e2;  % some limit to the control input
% PD Control
% T = @(e, Q_d) max(-torque_limit, min(torque_limit,-KP*e - KD*Q_d));

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
% Reference Model
M_hat = @(q1, q2) 0*M(q1, q2); % assuming perfect knowledge of the inertia matrix
C_hat = @(theta1,theta2, theta1_d,theta2_d, b1_hat, b2_hat) 0*Lagrangian_C_manual(I1,m1,l1, I2,m2,l2,  theta1,theta2, theta1_d,theta2_d, b1_hat, b2_hat); % don't know damping terms
K_hat = @(k1_hat, k2_hat) 0*Lagrangian_K(k1_hat,k2_hat);
G_hat = @(k1_hat, k2_hat) 0*Lagrangian_G(k1_hat,k2_hat,theta1_0,theta2_0);
F_hat = @(tau1, tau2) [tau1; tau2];

a_hat_d = @(R,Y,s1) -R*Y*s1;
a_hat_current = zeros(4,1);
Ahat = zeros(4,1);
Ahat_true = [3.3, 0.97, 1.04, 0.6];





% Adaptive control law
R = diag([0.03, 0.05, 0.1, 0.3]);
T = @(Y, a_hat, K_D, s) Y*a_hat - K_D*s;
hrw = 20*eye(m_inputs); % Hurwitz constant
Qr_d = @(Q_target_d, e)Q_target_d - hrw*e; %reference velocity
Qr_dd = @(Q_target_dd, ed)Q_target_dd - hrw*ed; % reference acceleration
s = @(Q_d, Qr_d) Q_d - Qr_d;

% Feed Forward
% use a PD Control, but also apply a torque term based on current state for
% springs
% T_FF = F


tau1 = q1_target(1); tau2 = q2_target(2);

% Constant
% T = @(e, Q_d) [0; 1];

Q_all = [];
Q_d_all = [];
T_all = [];
q1_target_t = [];
q2_target_t = [];
Ahat_all = [];
i_t = 1;
for t = ts
    % control law
    % % PD Control
    Qdes = Q_target(i_t);
    Qtilde = e(Q, Qdes); % error in position, velocity
    Qr = Qdes(:,2:3)' - hrw*Qtilde(:,1:2)'; % this starts with a first derivative
    
    %Calculate current reference velocity, reference acceleration
%     qr_d_current = Qr_d(Qdes(:,2), Qtilde(:,1));
%     qr_dd_current = Qr_dd(Qdes(:,3), Qtilde(:,2));
    qr_d_current = Qr(:,1);
    qr_dd_current = Qr(:,2);
    qr1_d = qr_d_current(1);
    qr2_d = qr_d_current(2);
    qr1_dd = qr_dd_current(1);
    qr2_dd = qr_dd_current(2);
    
    % Model Adaptation
    % Velocity error term
    s_current = s(Q(:,2), qr_d_current);
    
    % Estimates of a
    Y = Y_func(qr1_dd, qr2_dd, qr1_d, qr2_d, q1_d, q2_d, q2);
    Y_t = transpose(Y);
    a_hat_d_current = a_hat_d(R,Y_t,s_current);
    a_hat_current = a_hat_current+a_hat_d_current*dt;
    T_current = T(Y, a_hat_current, KD, s_current);
    tau1 = T_current(1);
    tau2 = T_current(2);
    
    % solve for accelerations (this is plant, do not change anything) 
    Q_dd = M(q1,q2) \ (F(tau1,tau2,Q(:,2)) - C(q1,q2,q1_d,q2_d)*Q(:,2) - K()*Q(:,1) - G());
    
    Q(:,2) = Q(:,2) + Q_dd * dt;
    q1_d = Q(1,2);
    q2_d = Q(2,2);
    
    Q = Q + Q(:,2) * dt;
    q1 = Q(1);
    q2 = Q(2);
    
    Q_all = [Q_all; Q(:,1)'];
    Q_d_all = [Q_d_all; Q(:,2)'];
    T_all = [T_all; T_current'];
    
    q1_target_t = [q1_target_t; q1_target(i_t)];
    q2_target_t = [q2_target_t; q2_target(i_t)];
    
    Ahat_all = [Ahat_all; a_hat_current'];
    
    i_t = i_t + 1;
end

m = 3;
n = 1;

subplot(m,n,1);
plot(ts, Q_all(:,1), 'rx', ts, Q_all(:,2), 'bo');
hold on; plot(ts, q1_target_t, 'r-.', ts, q2_target_t, 'b-.'); hold off
title('Joint Positions');
legend('q1', 'q2')

subplot(m,n,2);
plot(ts, T_all(:,1), 'rx', ts, T_all(:,2), 'bo')
title('Torque Input');
legend('t1', 't2')

subplot(m,n,3);
plot(ts, Ahat_all(:,1), 'rx', ts, ones(length(ts))*Ahat_true(1), 'bo')
title('Parameter accuracy')
legend('a1')