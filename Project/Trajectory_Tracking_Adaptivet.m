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

q1_target = q1_target_func(ts');
q2_target = q2_target_func(ts');

% q1_target = [q1_target_func(ts(1:100)'); q1_target_func(ts(100))*ones(length(ts(101:end)),1)];
% q2_target = [q2_target_func(ts(1:100)'); q2_target_func(ts(100))*ones(length(ts(101:end)),1)];

q1_target_d = [0; diff(q1_target)];
q1_target_dd = [0; diff(q1_target_d)];
q2_target_d = [0; diff(q2_target)];
q2_target_dd = [0; diff(q2_target_d)];

Q_target = @(i_t) [q1_target(i_t); q2_target(i_t)];

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %position error
torque_limit = 1e2;  % some limit to the control input
clamp_func = @(t) max(-torque_limit, min(torque_limit, t));
tau1 = 0; tau2 = 0;

% Reference Model
M_hat = M; % assuming perfect knowledge of the inertia matrix
C_hat = @(theta1,theta2, theta1_d,theta2_d, b1_hat, b2_hat) Lagrangian_C_manual(I1,m1,l1, I2,m2,l2,  theta1,theta2, theta1_d,theta2_d, b1_hat, b2_hat); % don't know damping terms
K_hat = @(k1_hat, k2_hat) Lagrangian_K(k1_hat,k2_hat);
G_hat = @(k1_hat, k2_hat) Lagrangian_G(k1_hat,k2_hat,theta1_0,theta2_0);
F_hat = @(tau1, tau2) [tau1; tau2];

a_hat = @(b1_hat, b2_hat, k1_hat, k2_hat) [b1_hat; b2_hat; k1_hat; k2_hat];
Ahat = zeros(4,1);
Y = @(theta1r, theta1r_d, theta2r, theta2r_d) [theta1r_d,     0,          theta1r,    0;
                                                0,            theta2r_d,  0,          theta2r];

LAMBDA = 20;
HRW = 20*eye(m_inputs); % Hurwitz constant
GAMMA = diag([0.03, 0.05, 0.1, 0.3]);
% Qtilde = @(Q, Qdes) Q - Qdes;
% Qr = @(Qdes, Qtilde) Qdes(2) - HRW*Qtilde(1);
s = @(Q_d, Qr) Q_d - Qr(1,:)';
% v = @(qdes_dd, Qtilde) qdes_dd - 2*lambda*Qtilde(2) - lambda^2*Qtilde(1);


Q_all = [];
Q_d_all = [];
T_all = [];
q1_target_t = [];
q2_target_t = [];
i_t = 1;
for t = ts
    Qdes = [q1_target(i_t),   q1_target_d(i_t), q1_target_dd(i_t);
            q2_target(i_t),   q2_target_d(i_t), q2_target_dd(i_t)];
    Qtilde = [Q, Q_d, Q_dd] - Qdes;
    Qr = Qdes(:,2:3)' - HRW*Qtilde(:,1:2)'; % this starts with a first derivative
    theta1r = Qr(1,1);
    theta2r = Qr(1,2);
    theta1r_d = Qr(2,1);
    theta2r_d = Qr(2,2);
    % control law
    % Model Adaptation
    Ahat_dot = -inv(GAMMA) * Y(theta1r, theta1r_d, theta2r, theta2r_d)'*Qtilde(:,2);
    Ahat = Ahat + Ahat_dot*dt; % update parameters
    b1_hat = Ahat(1);
    b2_hat = Ahat(2);
    k1_hat = Ahat(3);
    k2_hat = Ahat(4);
    M_hat_current = M_hat(q1, q2);
    C_hat_current = C_hat(q1,q2, q1_d,q2_d, b1_hat, b2_hat);
    K_hat_current = K_hat(k1_hat, k2_hat);
    G_hat_current = G_hat(k1_hat, k2_hat);
    F_hat_current = F_hat(tau1, tau2);
    
    % Feedforward Torque Input
    T_current = M_hat(q1, q2)*Qr(2,:)' + ...
        C_hat(q1,q2, q1_d,q2_d, b1_hat, b2_hat)*Qr(1,:)' + ...
        K_hat(k1_hat, k2_hat)*Qdes(:,1) + ...
        G_hat(k1_hat, k2_hat) - ...
        KD*s(Q_d,Qr);
    tau1 = T_current(1);
    tau2 = T_current(2);
%     tau1 = clamp_func(T_current(1));
%     tau2 = clamp_func(T_current(2));
    
    % solve for accelerations
    Q_dd = M(q1,q2) \ (F(tau1,tau2,q1_d,q2_d) - C(q1,q2,q1_d,q2_d)*Q_d - K()*Q - G());
    
    Q_d = Q_d + Q_dd * dt;
    q1_d = Q_d(1);
    q2_d = Q_d(2);
    
    Q = Q + Q_d * dt;
    q1 = Q(1);
    q2 = Q(2);
    
    Q_all = [Q_all; Q'];
    Q_d_all = [Q_d_all; Q_d'];
    T_all = [T_all; T_current'];
%     T_all = [T_all; [tau1, tau2]];
    
    q1_target_t = [q1_target_t; q1_target(i_t)];
    q2_target_t = [q2_target_t; q2_target(i_t)];
    
    i_t = i_t + 1;
end

m = 2;
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