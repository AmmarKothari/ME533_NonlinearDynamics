clear all
 % % % PUT ALL HAND FILES INTO A DIFFERENT FOLDER
 % % % SEPERATE THINGS MORE -- DIFFICULT TO BE PULLING FROM SAME MATRICES
% Setup
f = figure(1);
clf(f);
ax = axes(f);

% Configuration
hand_config1()
system_params_1()
Hand_Lagrangian_setup()
Planner_config1()

% Draw Gripper
g.draw(ax);
obj.draw(ax);
axis equal

% Draw Contact Points
hold on
plot(ax, contact_points(:,1), contact_points(:,2), 'o')
hold off

% Trajectory
% solve for angles to touch end to contact points
% minimize squared error
start_alphas = g.get_alphas();
end_pts = g.endPoints();
alphas = g.invKin(contact_points);

% Joint Space -- Linear Interp
% alpha_path = linearAlphaPath(start_alphas, alphas, 100);
runPlanner()
alpha_path = stomp_path;

% Work Space
xy_path = [];
for i = 1:length(alpha_path)
    g_temp = g.calc_poses(alpha_path(i,:));
    xy_path = [xy_path; g_temp.endPoints()];
end

% Plot Workspace
hold on; plot(xy_path(:,1), xy_path(:,2), 'rx'); hold off;



% % % % Adaptive Control Stuff % % % %
% assume that both fingers have the same model
q_target = stomp_path;
q_target_d = [zeros(1,size(q_target,2)); diff(q_target)];
q_target_dd = [zeros(1,size(q_target,2)); diff(q_target_d)];

Q_target = @(i_t) [q_target(i_t,:)', q_target_d(i_t,:)', q_target_dd(i_t,:)'];

% set initial conditions if necessary
Q = [start_alphas', zeros(Q_INPUTS,2)];

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
KD = 100*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %errors [position, velocity, acceleration]
torque_limit = 1e2;  % some limit to the control input
clamp_func = @(t) max(-torque_limit, min(torque_limit, t));
tau1 = 0; tau2 = 0;

% Reference Model -- assuming both fingers are the same
M_hat = @(Q) M2(Q); % assuming perfect knowledge of the inertia matrix
C_hat = @(Q, b1_hat, b2_hat) Lagrangian_C_manual(I1,m1,l1, I2,m2,l2,  Q, b1_hat, b2_hat); % don't know damping terms
% K_hat = @(k1_hat, k2_hat) K2(Q) (k1_hat,k2_hat);
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
Qr = @(Qdes, e) Qdes(:,2:end) - hrw*e(:,1:end-1); %trajectory references
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
    
%     qr1_d = Qr_current(1,1);
%     qr2_d = Qr_current(2,1);
%     qr1_dd = Qr_current(1,2);
%     qr2_dd = Qr_current(2,2);
    
    % Velocity error term
    s_current = s(Q(:,2), Qr_current(:,1));
    
    % Feedforward Torque Input
    % the dimensions of Qr_current were wrong in the 2 link example, but
    % they seem to work! -- correct version is currently in here
    T_current = M_hat(Q)*Qr_current(:,2) + ...
        C_hat(Q, b1_hat, b2_hat)*Qr_current(:,1) + ...
        K_hat(k1_hat, k2_hat)*Qdes(:,1) + ...
        G_hat(k1_hat, k2_hat) - ...
        KD*s(Q(:,2),Qr_current);
    tau1 = T_current(1);
    tau2 = T_current(2);
%     tau1 = clamp_func(T_current(1));
%     tau2 = clamp_func(T_current(2));
    
    % solve for accelerations -- Plant
    Q_dd = M(q1,q2) \ (F(tau1,tau2,Q(:,2)) - C(q1,q2,q1_d,q2_d)*Q(:,2) - K()*Q(:,1) - G());
    
    % control law
    % Model Adaptation
    Y = Y_func(qr1_d, qr1_dd, qr2_d, qr2_dd);
    a_hat_d_current = a_hat_d(R,Y,s_current);
    Ahat_current = Ahat_current+a_hat_d_current*dt;
    b1_hat = Ahat_current(1);
    b2_hat = Ahat_current(2);
    k1_hat = Ahat_current(3);
    k2_hat = Ahat_current(4);
    M_hat_current = M_hat(q1, q2);
    C_hat_current = C_hat(q1,q2, q1_d,q2_d, b1_hat, b2_hat);
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
