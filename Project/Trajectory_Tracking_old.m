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

Q_target = @(i_t) [q1_target(i_t); q2_target(i_t)];
Q_target_d = @(i_t) [q1_target_d(i_t); q2_target_d(i_t)];
Q_target_dd = @(i_t) [q1_target_dd(i_t); q2_target_dd(i_t)];

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %position error
torque_limit = 1e2;  % some limit to the control input

% Adaptive control law
R = [0.03 0 0 0; 0 0.05 0 0; 0 0 0.1 0; 0 0 0 0.3];
a_hat_d = @(R,Y,s1)-R*Y*s1;
T = @(Y, a_hat, K_D, s) Y*a_hat - K_D*s;
hrw = 20*eye(m_inputs); % Hurwitz constant
ed = @(Q_d, Q_target_d) Q_d - Q_target_d; % velocity error
Qr_d = @(Q_target_d, e)Q_target_d - hrw*e; %reference velocity
Qr_dd = @(Q_target_dd, ed)Q_target_dd - hrw*ed; % reference acceleration
s = @(Q_d, Qr_d) Q_d - Qr_d;

% Feed Forward
% use a PD Control, but also apply a torque term based on current state for
% springs
% T_FF = F

LAMBDA = 20;
HRW = 20*eye(m_inputs); % Hurwitz constant
Qtilde = @(Q, Qdes) Q - Qdes;
Qr = @(Qdes, Qtilde) Qdes(2) - HRW*Qtilde(1);
s = @(Q, Qr) Q(2) - Qr(2);

tau1 = q1_target(1); tau2 = q2_target(2);

% Constant
% T = @(e, Q_d) [0; 1];

Q_all = [];
Q_d_all = [];
T_all = [];
q1_target_t = [];
q2_target_t = [];
i_t = 1;
for t = ts
    % control law
    % % PD Control
    
    % error in position, velocity
    e_current = e(Q, Q_target(i_t));
    e_current_d = ed(Q_d, Q_target_d(i_t));
    
    %Calculate current reference velocity, reference acceleration
    qr_d_current = Qr_d(Q_target_d(i_t), e_current);
    qr_dd_current = Qr_dd(Q_target_dd(i_t), e_current_d);
    qr1_d = qr_d_current(1);
    qr2_d = qr_d_current(2);
    qr1_dd = qr_dd_current(1);
    qr2_dd = qr_dd_current(2);
    
    % Velocity error term
    s_current = s(Q_d, qr_d_current);
    
    % Estimates of a
    Y = Y_func(qr1_dd, qr2_dd, qr1_d, qr2_d, q1_d, q2_d, q2);
    Y_t = transpose(Y);
    a_hat_d_current = a_hat_d(R,Y_t,s_current);
    a_hat_current = a_hat_current+a_hat_d_current*dt;
    T_current = T(Y, a_hat_current, KD, s_current);
    tau1 = T_current(1);
    tau2 = T_current(2);
    
    % solve for accelerations (this is plant, do not change anything) 
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