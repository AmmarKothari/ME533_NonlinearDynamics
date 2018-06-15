clear all
addpath('..');
% Setup
f = figure(1);
clf(f);
ax = axes(f);

% Configuration
hand_config1()
% g = g.calc_poses([0, -pi/3, 0, -pi/4, pi/4]);
system_params_1()
Hand_Lagrangian_setup()
Planner_config1()

% Trajectory
% solve for angles to touch end to contact points
% minimize squared error
start_alphas = g.get_alphas();
end_pts = g.endPoints();
alphas = g.invKin(contact_points);

% Warm up pahts
% warm_up_paths_1 = start_alphas;
% for t_warm = 0:dt:1
%     warm_up_paths_1 = [warm_up_paths_1; warm_up_paths_1(1,:)+sin(t_warm)];
% end
% warm_up_paths_2 = warm_up_paths_1(end,:);
% for t_warm = 0:dt:1
%     warm_up_paths_2 = [warm_up_paths_2; warm_up_paths_2(1,:)+cos(t_warm)*(rem(t_warm,0.1)^2)];
% end
% 
% warm_up_paths = [warm_up_paths_1; warm_up_paths_2];

% Joint Space -- Linear Interp
% alpha_path = linearAlphaPath(start_alphas, alphas, 100);
runPlanner()
% alpha_path = [warm_up_paths; stomp_path];
alpha_path = stomp_path;


% Pursuit Constants
TRAJECTORY_EPSILON = [1e-3, 1e-2];
DEADZONE = 0.01;


% % % % Adaptive Control Stuff % % % %
% assume that both fingers have the same model
q_target = alpha_path;
q_target_d = [zeros(1,size(q_target,2)); diff(q_target)];
q_target_dd = [zeros(1,size(q_target,2)); diff(q_target_d)];

Q_target = @(i_t) [q_target(i_t,:)', q_target_d(i_t,:)', q_target_dd(i_t,:)'];

% set initial conditions if necessary
Q = [start_alphas', zeros(Q_INPUTS,2)];

% % % % % % % % % % % % % 
% Control law
% % % % % % % % % % % % % 
KD = 200*eye(m_inputs);
KP = 20*KD;
e = @(Q, Q_target) Q - Q_target; %errors [position, velocity, acceleration]
torque_limit = 1e2;  % some limit to the control input
clamp_func = @(t) max(-torque_limit, min(torque_limit, t));
tau1 = 0; tau2 = 0;

% % % Reference Model -- assuming both fingers are the same
M_hat = @(Q) M2(Q); % assuming perfect knowledge of the inertia matrix
% don't know damping terms
C_hat_Rfing = @(Q,Rfing) Lagrangian_C_manual(Rfing.I1,Rfing.m1,Rfing.l1,...
                            Rfing.I2,Rfing.m2,Rfing.l2,...
                            Q(2,1), Q(3,1), ...
                            Q(2,2), Q(3,2), ...
                            Rfing.b1, Rfing.b2);
C_hat_Lfing = @(Q,Lfing) Lagrangian_C_manual(Lfing.I1,Lfing.m1,Lfing.l1,...
                            Lfing.I2,Lfing.m2,Lfing.l2,...
                            Q(4,1), Q(5,1), ...
                            Q(4,2), Q(5,2), ...
                            Lfing.b1, Lfing.b2);
C_hat = @(Q, Rfing, Lfing) CreateDiagnolishMatrix(C_hat_Rfing(Q,Rfing), C_hat_Lfing(Q,Lfing));
% C_hat = @(Q, Rfing, Lfing) [zeros(2,1), C_hat_Rfing(Q,Rfing), zeros(2,2);
%                             zeros(2,1), zeros(2,2), C_hat_Lfing(Q,Lfing)];
                        

KR = @(Rfing) Lagrangian_K_manual(Rfing.k1,Rfing.k2);
KL = @(Lfing) Lagrangian_K_manual(Lfing.k1,Lfing.k2);
K_hat = @(Rfing,Lfing) CreateDiagnolishMatrix(KR(Rfing), KL(Lfing));
% K_hat = @(Rfing, Lfing) [zeros(2,1), KR(Rfing), zeros(2,2);
%         zeros(2,1), zeros(2,2), KL(Lfing)];

FR = @(tau1, tau2) [tau1; tau2];
FL = @(tau3, tau4) [tau3; tau4];
F_hat = @(Tau) [Tau(1);
                FR(Tau(2), Tau(3));
                FL(Tau(4), Tau(5))];
            
G_ = @(fing) Lagrangian_G(fing.k1,fing.k2,fing.theta1_0,fing.theta2_0);
G_hat = @(Rfing_hat, Lfing_hat) [0;
                                G_(Rfing_hat);
                                G_(Lfing_hat)];



a_hat_d = @(R, Y, s) -inv(R) * Y'*s; % this should be inv(R)?
Rfing_hat = Rfing;
Lfing_hat = Lfing;
Ahat_true = [Rfing.b1, Rfing.b2, Rfing.k1, Rfing.k2, Lfing.b1, Lfing.b2, Lfing.k1, Lfing.k2];
Ahat_current = 0*Ahat_true';
Rfing_hat.b1_hat = Ahat_current(1);
Rfing_hat.b2_hat = Ahat_current(2);
Rfing_hat.k1_hat = Ahat_current(3);
Rfing_hat.k2_hat = Ahat_current(4);
Lfing_hat.b1_hat = Ahat_current(5);
Lfing_hat.b2_hat = Ahat_current(6);
Lfing_hat.k1_hat = Ahat_current(7);
Lfing_hat.k2_hat = Ahat_current(8);
Y_func = @(Rfing, Lfing) [0,0,0,0,          0,0,0,0;
                            Rfing.theta1r_d,     0,          Rfing.theta1r,    0, 0, 0, 0, 0;
                          0,            Rfing.theta2r_d,  0,          Rfing.theta2r, 0, 0, 0, 0;
                          0, 0, 0, 0, Rfing.theta1r_d,     0,          Rfing.theta1r, 0;
                          0, 0, 0, 0, 0,            Rfing.theta2r_d,  0,          Rfing.theta2r];
                                
% Adaptive control law
R = diag([0.03, 0.05, 0.1, 0.3, 0.03, 0.05, 0.1, 0.3]);
hrw = 20*eye(m_inputs); % Hurwitz constant
Qr = @(Qdes, e) Qdes(:,2:end) - hrw*e(:,1:end-1); %trajectory references
s = @(Q_d, Qr_d) Q_d - Qr_d;

Q_all = [];
Q_d_all = [];
T_all = [];
Qdes_all = [];
Ahat_all = [];
i_t = 1;
t = 0; ts = [];
trajectory_error_all =[];
% for t = ts
while i_t <= size(alpha_path,1)
    Qdes = Q_target(i_t);
    
    % error in position, velocity
    e_current = e(Q, Qdes);
    
    %Calculate current reference velocity, reference acceleration
    Qr_current = Qr(Qdes, e_current);
    Rfing.theta1r = Qr_current(2,1);
    Rfing.theta1r_d = Qr_current(2,2);
    Rfing.theta2r = Qr_current(3,1);
    Rfing.theta2r_d = Qr_current(3,2);
    Lfing.theta1r = Qr_current(4,1);
    Lfing.theta1r_d = Qr_current(4,2);
    Lfing.theta2r = Qr_current(5,1);
    Lfing.theta2r_d = Qr_current(5,2);
    
    
    
    % Velocity error term
    s_current = s(Q(:,2), Qr_current(:,1));
    
    % Feedforward Torque Input
    % the dimensions of Qr_current were wrong in the 2 link example, but
    % they seem to work! -- correct version is currently in here
    T_current = M_hat(Q)*Qr_current(:,2) + ...
        C_hat(Q, Rfing_hat, Lfing_hat)*Qr_current(:,1) + ...
        K_hat(Rfing_hat, Lfing_hat)*Qdes(:,1) + ...
        KD*s_current;
%     tau1 = clamp_func(T_current(1));
    
    % solve for accelerations -- Plant
    p1 = M2(Q);
    p2 = F2(T_current) - C2(Q)*Q(:,2) - K2()*Q(:,1) - G2();
    Q_dd = p1(2:end,2:end) \ p2(2:end); % removing the unactuated joint
    Q_dd = [ 0; Q_dd]; % add zero action for that joint
    Q_dd(isnan(Q_dd)) = 0; % deals with 
    
    % control law
    % Model Adaptation
    Y = Y_func(Rfing, Lfing);
    trajectory_error = norm(e_current(:,1));
    trajectory_error_all = [trajectory_error_all; trajectory_error];
    if trajectory_error < DEADZONE % DEADZONE!
        a_hat_d_current = 0*Ahat_current;
    else
        a_hat_d_current = a_hat_d(R,Y,s_current);
    end
    Ahat_current = Ahat_current+a_hat_d_current*dt;
    Rfing_hat.b1_hat = Ahat_current(1);
    Rfing_hat.b2_hat = Ahat_current(2);
    Rfing_hat.k1_hat = Ahat_current(3);
    Rfing_hat.k2_hat = Ahat_current(4);
    Lfing_hat.b1_hat = Ahat_current(5);
    Lfing_hat.b2_hat = Ahat_current(6);
    Lfing_hat.k1_hat = Ahat_current(7);
    Lfing_hat.k2_hat = Ahat_current(8);
    
    M_hat_current = M_hat(Q);
    C_hat_current = C_hat(Q, Rfing_hat, Lfing_hat);
    K_hat_current = K_hat(Rfing_hat, Lfing_hat);
    G_hat_current = G_hat(Rfing_hat, Lfing_hat);
    F_hat_current = F_hat(T_current);
    
    % for some reason accelerations are opposite of what they should be?
    Q(:,2) = Q(:,2) + -Q_dd * dt;
%     Q(:,2) = Q(:,2) + Q_dd.*[0;1;-1;1;-1]*dt;
    
    Q(:,1) = Q(:,1) + Q(:,2) * dt;
    
    Q_all = [Q_all; Q(:,1)'];
    Q_d_all = [Q_d_all; Q(:,2)'];
    T_all = [T_all; T_current'];
    
    Qdes_all = [Qdes_all; Qdes(:,1)'];
    
    Ahat_all = [Ahat_all; Ahat_current'];
    
    t = t + dt;
    ts = [ts; t];
    
    % Pure Pursuit Trajectory Following
    if i_t == size(alpha_path,1)
        trajectory_epsilon = TRAJECTORY_EPSILON(1);
    else
        trajectory_epsilon = TRAJECTORY_EPSILON(2);
    end
    if trajectory_error < trajectory_epsilon
        i_t = i_t + 1;
    end
    if rem(i_t,2) == 0
        col = {'rx', 'bo'};
    else
        col = {'gx', 'ko'};
    end
       
%     plot(ts, Q_all(:,2)-Qdes_all(:,2), col{1},...
%         ts, Q_all(:,3)-Qdes_all(:,3), col{2},...
%         ts, Q_all(:,4)-Qdes_all(:,4), col{1},...
%         ts, Q_all(:,5)-Qdes_all(:,5), col{2}...
%         );
    plot(ts, trajectory_error_all, 'rx')
    drawnow limitrate
end

m = 3;
n = 1;

% subplot(m,n,1);
% plot(ts, Q_all(:,2), 'rx', ts, Q_all(:,3), 'bo');
% hold on; plot(ts, Qdes_all(:,1), 'r-.', ts, Qdes(:,2), 'b-.'); hold off
% title('Joint Positions');

% legend('q1', 'q2')
% figure(2)
% subplot(m,n,1);
plot(ts, Q_all(:,2)-Qdes_all(:,2), 'rx', ts, Q_all(:,3)-Qdes_all(:,3), 'bo');
% hold on; plot(ts, , 'r-.', ts, , 'b-.'); hold off
% title('Position Error');
% legend('q1', 'q2')

% subplot(m,n,2);
% plot(ts, T_all(:,1), 'rx', ts, T_all(:,2), 'bo')
% title('Torque Input');
% legend('t1', 't2')
% 
% subplot(m,n,3)
% plot(ts, Ahat_all);
% hold on;
% plot(ts, ones(length(ts),1)*Ahat_true, ':');
% hold off;
% legend('a1_{hat}', 'a2_{hat}', 'a3_{hat}', 'a4_{hat}', 'a1', 'a2', 'a3', 'a4')
% title('Parameter Values')

% Desired End Piont Trajectory
xy_path = [];
for i = 1:length(alpha_path)
    g_temp = g.calc_poses(alpha_path(i,:));
    xy_path = [xy_path; g_temp.endPoints()];
end

% Actual End Point Trajectory
xy_actual_path = [];
for i = 1:size(Q_all,1)
    g_temp = g.calc_poses(Q_all(i,:));
    xy_actual_path = [xy_actual_path; g_temp.endPoints()];
end
% Description = "Tight bounds on following trajectory. Different Starting point.";
Description = "Tight bounds on following trajectory.";
save('output_traj_1.mat', 'Q_all', 'Qdes_all', 'trajectory_error_all', 'xy_path', 'xy_actual_path',...
    'contact_points', 'g', 'obj', 'ts', 'Ahat_all', 'Ahat_true', 'TRAJECTORY_EPSILON', 'DEADZONE', ...
    'Description')

%% Generate Animation
clear all
load('output_traj_1.mat')
f = figure(2);
clf(f);
ax = axes(f);
    
gif_name = 'tight.gif';

% Plot showing actual and desired finger paths -- PAPER!
plot(xy_path(:,1), xy_path(:,2), 'rx')
hold on;
plot(xy_actual_path(:,1), xy_actual_path(:,2), 'bo')
hold off
% legend('Desired', 'Actual')
xlabel('X Pos')
ylabel('Y Pos')
title('Desired and Actual End Point Trajectories')
axis tight

for i = 1:50:size(Q_all,1)
    cla(f);
    g = g.calc_poses(Q_all(i,:));
    % Draw Gripper
    g.draw(ax);
    
    % Draw Object
    obj.draw(ax);
    
    % Draw Contact Points
    hold on
    plot(ax, contact_points(:,1), contact_points(:,2), 'o')
    plot(xy_path(:,1), xy_path(:,2), 'rx')
    hold off
    
    axis equal
%     drawnow limitrate
    drawnow
    frame = getframe(f);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im,256);
    if i == 1 
        imwrite(imind,cm,gif_name,'gif', 'Loopcount',1); 
    else 
        imwrite(imind,cm,gif_name,'gif','WriteMode','append'); 
    end 
end


% 
% % Plot Workspace
% hold on; plot(xy_path(:,1), xy_path(:,2), 'rx'); hold off;

%% Plots for Paper
clear all
load('output_traj_1.mat')
f = figure(2);
clf(f);
ax = axes(f);

    
% Plot showing actual and desired finger paths
plot(xy_path(:,1), xy_path(:,2), 'ro')
hold on; plot(xy_actual_path(:,1), xy_actual_path(:,2), 'bx'); hold off;
legend('Desired', 'Actual')
xlabel('X Pos')
ylabel('Y Pos')
title('Desired and Actual End Point Trajectories -- Tight Error Bounds')
axis tight
saveas(gcf, 'TrajectoryEndpointError_Tight.png');

% Plot showing joint angle errors
plot(ts, Q_all(:,2)-Qdes_all(:,2), 'rx', ...
    ts, Q_all(:,3)-Qdes_all(:,3), 'bo',...
    ts, Q_all(:,4)-Qdes_all(:,4), 'g^',...
    ts, Q_all(:,5)-Qdes_all(:,5), 'k*'...
    )
legend('R-Proximal', 'R-Distal', 'L-Proximal', 'L-Distal')
title('Error in Joint Angles -- Tight Error Bounds')
xlabel('Time(s)')
ylabel('Joint Angle Error (radians)');
saveas(gcf, 'JointError_Tight.png');

% Plot Unknowns
col = jet(8);
plot_step = 10;
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,1)-Ahat_true(1), 'color',col(1,:), 'marker', 'x', 'LineStyle','none')
hold on
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,2)-Ahat_true(2), 'color',col(2,:), 'marker', 'o', 'LineStyle','none')
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,5)-Ahat_true(5), 'color',col(3,:), 'marker', '^', 'LineStyle','none')
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,6)-Ahat_true(6), 'color',col(4,:), 'marker', '*', 'LineStyle','none')
hold off
title('Springs: Parameter Estimation Error')
xlabel('Time')
ylabel('Parameter Error')
legend('R-Proximal', 'R-Distal', 'L-Proximal', 'L-Distal')
saveas(gcf, 'Spring_ParamError_Tight.png')

plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,3)-Ahat_true(3), 'color',col(1,:), 'marker', 'x', 'LineStyle','none')
hold on
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,4)-Ahat_true(4), 'color',col(2,:), 'marker', 'o', 'LineStyle','none')
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,7)-Ahat_true(7), 'color',col(3,:), 'marker', '^', 'LineStyle','none')
plot(ts(1:plot_step:end), Ahat_all(1:plot_step:end,8)-Ahat_true(8), 'color',col(4,:), 'marker', '*', 'LineStyle','none')
hold off
title('Dampers: Parameter Estimation Error')
xlabel('Time')
ylabel('Parameter Error')
legend('R-Proximal', 'R-Distal', 'L-Proximal', 'L-Distal')
saveas(gcf, 'Damper_ParamError_Tight.png')



% Plots for loose bounds
clear all
load('output_traj_2.mat')
f = figure(2);
clf(f);
ax = axes(f);

    
% Plot showing actual and desired finger paths
plot(xy_path(:,1), xy_path(:,2), 'ro')
hold on; plot(xy_actual_path(:,1), xy_actual_path(:,2), 'bx'); hold off;
legend('Desired', 'Actual')
xlabel('X Pos')
ylabel('Y Pos')
title('Desired and Actual End Point Trajectories -- Loose Error Bounds')
axis tight
saveas(gcf, 'TrajectoryEndpointError_Loose.png');

% Plot showing joint angle errors
plot(ts, Q_all(:,2)-Qdes_all(:,2), 'rx', ...
    ts, Q_all(:,3)-Qdes_all(:,3), 'bo',...
    ts, Q_all(:,4)-Qdes_all(:,4), 'g^',...
    ts, Q_all(:,5)-Qdes_all(:,5), 'k*'...
    )
legend('R-Proximal', 'R-Distal', 'L-Proximal', 'L-Distal')
title('Error in Joint Angles -- Loose Error Bounds')
xlabel('Time(s)')
ylabel('Joint Angle Error (radians)');
saveas(gcf, 'JointError_Loose.png');