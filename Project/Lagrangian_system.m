% system setup using lagrangian equations that have been derived

m_inputs = 2;
I_func = @(m, l) 1/12*m*l^2;

% physical parameters of the arm
m1 = 1;
l1 = 1;
m2 = 2;
l2 = 1.2;

I1 = I_func(m1, l1);
I2 = I_func(m2, l2);

lc1 = l1/2;
lc2 = l2/2;

k1 = 3.0; k2 = 1.0;
b1 = 1; b2 = 2;
theta1_0 = 0; % resting position of spring
theta2_0 = 0; 

% Inertia Matrix
M = @(q1, q2) Lagrangian_M(I1,I2,l1,l2,m1,m2,q2);

% Coriolis and Damping Matrix
C = @(q1, q2, q1_d, q2_d) Lagrangian_C(l1, l2, m2, q2, q1_d, q2_d);

% Spring
K = @() Lagrangian_K(k1,k2);
K_hat = @(k1_hat, k2_hat) Lagrangian_K(k1,k2);

% Potential
% for current scenario, this is where the resting length of the spring is
% kept
G = @() Lagrangian_G(k1,k2,theta1_0,theta2_0);

% Generalized Forces
F = @(tau1,tau2,q1_d,q2_d) Lagrangian_Q(b1,b2,tau1,tau2,q1_d,q2_d);

% Joint Angles
q1 = 0; q2 = 0;
q1_d = 0; q2_d = 0;

Q = [q1; q2];
Q_d = [q1_d; q2_d];

% target Joint angle -- for position tracking
q1_target = pi/3; q2_target = pi/2;
Q_target = [q1_target; q2_target];

% Control law
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q , Q_target) Q - Q_target;
torque_limit = 1e5;  % some limit to the control input
% T = @(e, Q_d) max(-torque_limit, min(torque_limit,-KP*e - KD*Q_d));
T = @(e, Q_d) [0; 1];
tau1 = 0; tau2 = 0;


dt = 1e-3; % this needs to be atleast 1e-3, 1e-4 takes forever to finish!
ts = 0:dt:1;
Q_all = [];
Q_d_all = [];
T_all = [];
for t = ts
    % control law
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