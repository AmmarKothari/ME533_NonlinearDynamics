A = readtable('bh_hello_complete.xlsx', 'Sheet', 'COMPLETE');

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

% now use them
new_XE = fnval(the_xpp, new_T);
new_YE = fnval(the_ypp, new_T);

new_DXDT = fnval( fndir(the_xpp, 1), new_T );
new_DYDT = fnval( fndir(the_ypp, 1), new_T );

%% Let's visualize the results
figure;
subplot(2,1,1)
    plot(A.T,     A.XE, 'k-'); hold on
    plot(new_T, new_XE, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('X (m)');
    legend({'ORIG','splined'});
subplot(2,1,2)
    plot(A.T,     A.YE, 'k-'); hold on
    plot(new_T, new_YE, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('Y (m)')
    legend({'ORIG','splined'});
    
figure;
subplot(2,1,1)
    plot(A.T,     A.DXDT, 'k-'); hold on
    plot(new_T, new_DXDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('X (m/sec)');
    legend({'ORIG','splined'});
subplot(2,1,2)
    plot(A.T,     A.DYDT, 'k-'); hold on
    plot(new_T, new_DYDT, '.r-'); axis('tight'); grid('on');
    xlabel('time (secs)'); ylabel('Y (m/sec)')
    legend({'ORIG','splined'});
    
%% write the results to a table

B.T = new_T;
B.XE = new_XE;
B.YE = new_YE;
B.DXDT = new_DXDT;
B.DYDT = new_DYDT;

new_tab = struct2table(B)

writetable(new_tab,'bh_hello_complete.xlsx','Sheet', 'COMP_SPLINE')