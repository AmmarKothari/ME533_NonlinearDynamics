A = readtable('bh_4dof_traj_data.xlsx', 'Sheet', 'RAW');

t_start = A.T(1);
t_end   = A.T(end);

new_T = [t_start:0.01:t_end]';

% Let's create some cubic splines

the_t = A.T;
the_x = [0; A.XE; 0;]; % NOTE the ZERO deriv defs at start and end
the_xpp = spline(the_t, the_x);

the_t = A.T;
the_y = [0; A.YE; 0;]; % NOTE the ZERO deriv defs at start and end
the_ypp = spline(the_t, the_y);

the_t = A.T;
the_z = [0; A.ZE; 0;]; % NOTE the ZERO deriv defs at start and end
the_zpp = spline(the_t, the_z);

% now use them
new_XE = fnval(the_xpp, new_T);
new_YE = fnval(the_ypp, new_T);
new_ZE = fnval(the_zpp, new_T);

new_DXDT = fnval( fndir(the_xpp, 1), new_T );
new_DYDT = fnval( fndir(the_ypp, 1), new_T );
new_DZDT = fnval( fndir(the_zpp, 1), new_T );

new_DDXDT = fnval( fndir(the_xpp, 2), new_T );
new_DDYDT = fnval( fndir(the_ypp, 2), new_T );
new_DDZDT = fnval( fndir(the_zpp, 2), new_T );

%% check if the path is outside our reachable workspace
R_vec  = sqrt(new_XE.^2 + new_YE.^2 + new_ZE.^2);
tf_vec = R_vec > 1.5;
if(any(tf_vec) )
    nnz(tf_vec)
   error('ERROR:  we have traj points outside our reachable workspace'); 
end

%% Let's visualize the results
figure;
plot3(new_XE, new_YE, new_ZE, '.b-');
grid on; xlabel X; ylabel Y; zlabel Z


% plot position versus TIME
figure;
% plot the POSITIONS    
subplot(3,3,1)
    plot(A.T,     A.XE, 'k-'); hold on
    plot(new_T, new_XE, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('X (m)');
    legend({'ORIG','splined'});
subplot(3,3,4)
    plot(A.T,     A.YE, 'k-'); hold on
    plot(new_T, new_YE, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('Y (m)')
    legend({'ORIG','splined'});
subplot(3,3,7)
    plot(A.T,     A.ZE, 'k-'); hold on
    plot(new_T, new_ZE, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('Z (m)')
    legend({'ORIG','splined'});
% plot the VELOCITIES    
subplot(3,3,2)
    plot(new_T, new_DXDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('VX (m/sec)');
    legend({'VX'});
subplot(3,3,5)
    plot(new_T, new_DYDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('VY (m/sec)')
    legend({'VY'});
subplot(3,3,8)
    plot(new_T, new_DZDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('VZ (m/sec)')
    legend({'VZ'});
% plot the ACCELERATIONS    
subplot(3,3,3)
    plot(new_T, new_DDXDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('AX (m/sec)');
    legend({'AX'});
subplot(3,3,6)
    plot(new_T, new_DDYDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('AY (m/sec)')
    legend({'AY'});
subplot(3,3,9)
    plot(new_T, new_DDZDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('AZ (m/sec)')
    legend({'AZ'});
    
%% write the results to a table

B.T = new_T;
B.XE = new_XE;
B.YE = new_YE;
B.ZE = new_ZE;
B.DXDT = new_DXDT;
B.DYDT = new_DYDT;
B.DZDT = new_DZDT;

new_tab = struct2table(B)

%writetable(new_tab,'bh_4dof_traj_data.xlsx','Sheet', 'REFINED')