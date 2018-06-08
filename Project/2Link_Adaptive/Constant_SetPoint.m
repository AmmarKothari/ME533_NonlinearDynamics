% system setup using lagrangian equations that have been derived

% setting up system parameters
system_params_1();

% setting up functions
Lagrangian_setup();

% target Joint angle -- for position tracking
q1_target = pi/3; q2_target = pi/2;
Q_target = [q1_target; q2_target];

% Control law
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q , Q_target) Q - Q_target;
torque_limit = 1e5;  % some limit to the control input
% PD Control
T = @(e, Q_d) max(-torque_limit, min(torque_limit,-KP*e - KD*Q_d));
tau1 = 0; tau2 = 0;

% Constant
% T = @(e, Q_d) [0; 1];

dt = 1e-3; % this needs to be atleast 1e-3, 1e-4 takes forever to finish!
ts = 0:dt:1;
Q_all = [];
Q_d_all = [];
T_all = [];
for t = ts
    % control law
    % % PD Control
    e_current = e(Q, Q_target);
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
    

end

m = 2;
n = 1;

subplot(m,n,1);
plot(ts, Q_all(:,1), 'rx', ts, Q_all(:,2), 'bo');
hold on; plot(ts, ones(length(ts))*q1_target, 'r-.', ts, ones(length(ts))*q2_target, 'b-.'); hold off
legend('q1', 'q2')

subplot(m,n,2);
plot(ts, T_all(:,1), 'rx', ts, T_all(:,2), 'bo')
legend('t1', 't2')