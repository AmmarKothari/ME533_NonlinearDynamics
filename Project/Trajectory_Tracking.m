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

% Control law
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %position error
torque_limit = 1e2;  % some limit to the control input
% PD Control
T = @(e, Q_d) max(-torque_limit, min(torque_limit,-KP*e - KD*Q_d));

% PD Control for adaptive control
hrw = 20*eye(m_inputs); % Hurwitz constant
ed = @(Q_d, Q_target_d) Q_d - Q_target_d; % velocity error
qr_d = @(q_target_d, e)q1_target_d - hrw*e; %reference velocity
qr_dd = @(q_target_dd, ed)q1_target_dd - hrw*ed; % Not sure that's how to get reference acceleration


%Evaluate Y (adaptive control), this should go to loop
e1 = e(q1, q1_target);
ed1 = ed(q1_d, q1_target_d);
qr1_d = qr_d(q1_target_d, e1);
qr1_dd = qr_dd(q1_target_dd, ed1);
e2 = e(q2, q2_target);
ed2 = ed(q2_d, q2_target_d);
qr2_d = qr_d(q2_target_d, e2);
qr2_dd = qr_dd(q2_target_dd, ed2);
Y = Y_func(qr1_dd, qr2_dd, qr1_d, qr2_d, q1_d, q2_d, q2);  

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
i_t = 1;
for t = ts
    % control law
    % % PD Control
    e_current = e(Q, Q_target(i_t));
    T_current = T(e_current, Q_d);
    tau1 = T_current(1);
    tau2 = T_current(2);
    
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