
%% read data from EXCEL file
data_T = readtable('bh_hello_complete.xlsx','Sheet','COMP_SPLINE');

% downsample - say keep every 10th sample
data_T = data_T(1:10:end, :);
N      = size(data_T,1);

% plot it
figure;
plot(data_T.XE, data_T.YE, '-b.');
axis tight; grid on; xlabel X; ylabel Y

%% Define the LINK lengths
L1 = 0.75;
L2 = 0.5;
L3 = 0.25;

%% set up the constrained optimization problem
opts_T         = optimset;
opts_T.Display = 'off';
    
Q_RES = [];  % container for collecting results

qa_lb = [ -pi;  -pi;   -pi]; % LOWER bounds for angles
qa_ub = [  pi;   pi;    pi]; % UPPER bounds for angles
qa    = [ 0.5;  0.5;   0.5]; % initial gues for 1st data point
for kk=1:N
    fprintf('\n .... kk=%4d of %4d',kk, N)    

    % the END EFFECTOR X,Y position
    tgt_eff_pos = [data_T.XE(kk), data_T.YE(kk);];
    
    % the cost function I want to minimize
    my_CF = @(q)( norm( tgt_eff_pos - ...
                        bh_xy_for_3link_planar(L1,L2,L3,q(1),q(2),q(3)) ...
                      ) ...  
                );

    % make my starting guess for the angular pose, the pose computed from
    % the previousrun - we'd expect the next solution to be "close by". 
    q0 = qa;
   
    % invoke our optimization solver:
    %x = fmincon(fun,  x0,A, b,Aeq,beq,    lb,    ub, nonlcon, opts)
    qa  = fmincon(my_CF,q0,[],[],[],[], qa_lb, qa_ub, [],      opts_T);

    % store the angular results
    Q_RES = [Q_RES; qa'];

end

%% plot the END EFFECTOR position
new_XY = bh_xy_for_3link_planar(L1,L2,L3, Q_RES(:,1), Q_RES(:,2), Q_RES(:,3))

figure;
    plot(data_T.XE,    data_T.YE,  '-b.');  hold on;
    plot(new_XY(:,1),  new_XY(:,2), '-r');  
    axis tight; grid on; xlabel X; ylabel Y

%% plot the joint angles
Q_degs_RES = (180/pi)*Q_RES;
T          = data_T.T;
figure;
subplot(3,1,1);  plot(T, Q_degs_RES(:,1), '-r'); 
                 grid on; xlabel TIME; ylabel('THETA_1 (degs)'); axis tight;
subplot(3,1,2);  plot(T, Q_degs_RES(:,2), '-r'); 
                 grid on; xlabel TIME; ylabel('THETA_2 (degs)'); axis tight;
subplot(3,1,3);  plot(T, Q_degs_RES(:,3), '-r'); 
                 grid on; xlabel TIME; ylabel('THETA_3 (degs)'); axis tight;

%% plot the orientation of the end effector
figure

EFF_ori_degs = Q_degs_RES(:,1) + Q_degs_RES(:,2) + Q_degs_RES(:,3);
plot(T, EFF_ori_degs, '-k');
                 grid on; xlabel TIME; ylabel('ang EFF'); axis tight;
