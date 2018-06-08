% system setup using lagrangian equations that have been derived
clear all;

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

Q_target = @(i_t) [q1_target(i_t), q1_target_d(i_t), q1_target_dd(i_t);
                   q2_target(i_t), q2_target_d(i_t), q2_target_dd(i_t)];
               
% set initial conditions if necessary

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
KD = 100*eye(m_inputs);
KP = 20*KD;
DEADZONE_ERROR_RANGE = 0.1;
e = @(Q, Q_target) Q - Q_target; %errors [position, velocity, acceleration]
torque_limit = 1e2;  % some limit to the control input
clamp_func = @(t) max(-torque_limit, min(torque_limit, t));
tau1 = 0; tau2 = 0;

% Reference Model
M_hat = @(Q) M(Q); % assuming perfect knowledge of the inertia matrix
C_hat = @(Q, b1_hat, b2_hat) Lagrangian_C_manual(I1,m1,l1, I2,m2,l2,  Q(1,1),Q(2,1), Q(1,2), Q(2,2), b1_hat, b2_hat); % don't know damping terms
K_hat = @(k1_hat, k2_hat) Lagrangian_K(k1_hat,k2_hat);
G_hat = @(k1_hat, k2_hat) Lagrangian_G(k1_hat,k2_hat,theta1_0,theta2_0);
F_hat = @(tau1, tau2) [tau1; tau2];

a_hat_d = @(R, Y, s) -R * Y'*s; % this should be inv(R)?
Ahat_current = zeros(4,1);
b1_hat = Ahat_current(1);
b2_hat = Ahat_current(2);
k1_hat = Ahat_current(3);
k2_hat = Ahat_current(4);
Ahat_true = [b1, b2, k1, k2];
Y_func = @(theta1r, theta1r_d, theta2r, theta2r_d) [theta1r_d,     0,          theta1r,    0;
                                                0,            theta2r_d,  0,          theta2r];
% Adaptive control law
R = diag([0.03, 0.05, 0.1, 0.3]);
hrw = 20*eye(m_inputs); % Hurwitz constant
Qr = @(Qdes, e) Qdes(:,2:3) - hrw*e(:,1:2); %trajectory references
s = @(Q_d, Qr_d) Q_d - Qr_d;

Q_all = [];
Q_d_all = [];
T_all = [];
q1_target_t = [];
q2_target_t = [];
Ahat_all = [];
i_t = 1;
for t = ts
    Qdes = Q_target(i_t);
    
    % error in position, velocity
    e_current = e(Q, Qdes);
    
    %Calculate current reference velocity, reference acceleration
    Qr_current = Qr(Qdes, e_current); 
    
    qr1_d = Qr_current(1,1);
    qr2_d = Qr_current(2,1);
    qr1_dd = Qr_current(1,2);
    qr2_dd = Qr_current(2,2);
    
    % Velocity error term
    s_current = s(Q(:,2), Qr_current(:,1));
    
    % Feedforward Torque Input
    T_current = M_hat(Q)*Qr_current(2,:)' + ...
        C_hat(Q, b1_hat, b2_hat)*Qr_current(1,:)' + ...
        K_hat(k1_hat, k2_hat)*Qdes(:,1) + ...
        G_hat(k1_hat, k2_hat) - ...
        KD*s(Q(:,2),Qr_current);
    tau1 = T_current(1);
    tau2 = T_current(2);
%     tau1 = clamp_func(T_current(1));
%     tau2 = clamp_func(T_current(2));
    
    % solve for accelerations -- Plant
    Q_dd = M(Q) \ (F(tau1,tau2,Q) - C(Q)*Q(:,2) - K()*Q(:,1) - G());
    
    % control law
    % Model Adaptation
    Y = Y_func(qr1_d, qr1_dd, qr2_d, qr2_dd);
    a_hat_d_current = a_hat_d(R,Y,s_current);
    % dead zone
    if norm(e_current(:,1)) < DEADZONE_ERROR_RANGE
        a_hat_d_current = 0;
    end
    Ahat_current = Ahat_current+a_hat_d_current*dt;
    b1_hat = Ahat_current(1);
    b2_hat = Ahat_current(2);
    k1_hat = Ahat_current(3);
    k2_hat = Ahat_current(4);
    M_hat_current = M_hat(Q);
    C_hat_current = C_hat(Q, b1_hat, b2_hat);
    K_hat_current = K_hat(k1_hat, k2_hat);
    G_hat_current = G_hat(k1_hat, k2_hat);
    F_hat_current = F_hat(tau1, tau2);
    
    Q(:,2) = Q(:,2) + Q_dd * dt;
    q1_d = Q(1,2);
    q2_d = Q(2,2);
    
    Q = Q + Q(:,2) * dt;
    q1 = Q(1);
    q2 = Q(2);
    
    Q_all = [Q_all; Q(:,1)'];
    Q_d_all = [Q_d_all; Q(:,2)'];
    T_all = [T_all; [tau1, tau2]];
    
    q1_target_t = [q1_target_t; q1_target(i_t)];
    q2_target_t = [q2_target_t; q2_target(i_t)];
    
    Ahat_all = [Ahat_all; Ahat_current'];
    
    i_t = i_t + 1;
end

m = 3;
n = 1;

% subplot(m,n,1);
% plot(ts, Q_all(:,1), 'rx', ts, Q_all(:,2), 'bo');
% hold on; plot(ts, q1_target_t, 'r-.', ts, q2_target_t, 'b-.'); hold off
% title('Joint Positions');
% legend('q1', 'q2')

subplot(m,n,1);
plot(ts, Q_all(:,1)-q1_target_t, 'rx', ts, Q_all(:,2)-q2_target_t, 'bo');
% hold on; plot(ts, , 'r-.', ts, , 'b-.'); hold off
title('Position Error');
legend('q1', 'q2')

subplot(m,n,2);
plot(ts, T_all(:,1), 'rx', ts, T_all(:,2), 'bo')
title('Torque Input');
legend('t1', 't2')

subplot(m,n,3)
plot(ts, Ahat_all);
hold on;
plot(ts, ones(length(ts),1)*Ahat_true, ':');
hold off;
legend('a1_{hat}', 'a2_{hat}', 'a3_{hat}', 'a4_{hat}', 'a1', 'a2', 'a3', 'a4')
title('Parameter Values')
