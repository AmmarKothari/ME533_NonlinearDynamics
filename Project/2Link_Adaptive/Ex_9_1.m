% Example 9.1 from Nonlinear Book
m_inputs = 2;

% physical parameters of the arm
m1 = 1;
l1 = 1;
me = 2;
% delta_e = pi/6;
delta_e = 0;
% I1 = .12;
I1 = 1/12*m1*l1^2;
lc1 = 0.5;
% Ie = 0.25;
lce = 0.6;
l2 = 1.2;
Ie = 1/12*me*l2^2;

% define matrices
a1 = I1 + m1*lc1^2 + Ie + me*lce^2 + me*l1^2;
a2 = Ie + me*lce^2;
a3 = me*l1*lce*cos(delta_e);
a4 = me*l1*lce*sin(delta_e);


H11 = @(q1,q2) a1 + 2*a3*cos(q2) + 2*a4*sin(q2);
H12 = @(q1,q2) a2 + a3*cos(q2) + a4*sin(q2);
H21 = @(q1,q2) H12(q1, q2);
H22 = @(q1,q2) a2;
h   = @(q1,q2) a2*sin(q2) - a4*cos(q2);

% Inertia Matrix
H = @(q1,q2) [H11(q1,q2), H12(q1,q2); H21(q1,q2), H22(q1,q2)];

% Coriolis and Damping Matrix
C = @(q1, q2, q1_d, q2_d) [-h(q1,q2)*q2_d, -h(q1,q2)*(q1_d + q2_d); h(q1,q2)*q1_d, 0];


% joint angles
% syms q1 q2 q1_d q2_d t1 t2
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
T = @(e, Q_d) max(-torque_limit, min(torque_limit,-KP*e - KD*Q_d));


dt = 1e-3; % this needs to be atleast 1e-3, 1e-4 takes forever to finish!
ts = 0:dt:1;
Q_all = [];
Q_d_all = [];
T_all = [];
for t = ts
    % control law
    e_current = e(Q, Q_target);
    T_current = T(e_current, Q_d);
    
    % solve for accelerations
    Q_dd = H(q1, q2) \ (T_current - C(q1, q2, q1_d, q2_d)*Q_d);
    
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

