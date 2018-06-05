%% POSTprocessing script for SIMULINK models:
%  a.) bh_4dof_LAGR_manipulator_CONTROL_TQ_FFWD
%
% Bradley Horton, bradley.horton@mathworks.com.au

%% check that a "TO_WORKSPACE" logging variable exists
if( 1==exist('simout_CMD_and_ACTUAL_XYZ') )
    % good the variable exists
else
    error('###_ERROR:  was expecting to find a BASE workspace variable called <simout_CMD_and_ACTUAL_XYZ>');
end

%% CLOSE any figures
close('all');

%% PLOT stuff
% ------------------------------------------------------------------------
% simout_CMD_and_ACTUAL_XYZ
% 
% simout_CMD_and_ACTUAL_XYZ = 
% 
%   struct with fields:
% 
%          time: [60001×1 double]
%       signals: [1×1 struct]
%     blockName: 'bh_4dof_LAGR_manipulator_CONTROL_TQ_FFWD/SL_VIEW/To Workspace'
% ------------------------------------------------------------------------
% simout_CMD_and_ACTUAL_XYZ.signals
% 
% ans = 
% 
%   struct with fields:
% 
%         values: [60001×6 double]
%     dimensions: 6
%          label: ''
% ------------------------------------------------------------------------

t           = simout_CMD_and_ACTUAL_XYZ.time;
vals        = simout_CMD_and_ACTUAL_XYZ.signals.values;
CMD         = vals(:,[1,2,3]);
PLANT       = vals(:,[4,5,6]);
err_sq_XYZ  = (CMD - PLANT).^2;
err_norm    = sqrt( sum(err_sq_XYZ,2) ); 
%--------------------------------------------------------------------------
% trajectory Comparison and error
figure;
hax(1) = subplot(3,2,1); 
hax(2) = subplot(3,2,2); 
hax(3) = subplot(3,2,3);
hax(4) = subplot(3,2,4); 
hax(5) = subplot(3,2,5); 
hax(6) = subplot(3,2,6); 

plot(hax(1), t,   CMD(:,1), '-r.'); hold(hax(1),'on');
plot(hax(1), t, PLANT(:,1), '-b.'); 
plot(hax(2), t,   CMD(:,1)-PLANT(:,1),'-k.');
ylabel(hax(1),'X (mm)');    title(hax(1),'X     (mm)', 'FontWeight', 'Bold');
ylabel(hax(2),'X err(mm)'); title(hax(2),'X err (mm)', 'FontWeight', 'Bold');

plot(hax(3), t,   CMD(:,2), '-r.'); hold(hax(3),'on');
plot(hax(3), t, PLANT(:,2), '-b.'); 
plot(hax(4), t,   CMD(:,2)-PLANT(:,2),'-k.'); 
ylabel(hax(3),'Y (mm)');    title(hax(3),'Y     (mm)', 'FontWeight', 'Bold');
ylabel(hax(4),'Y err(mm)'); title(hax(4),'Y err (mm)', 'FontWeight', 'Bold');

plot(hax(5), t,   CMD(:,3), '-r.'); hold(hax(5),'on');
plot(hax(5), t, PLANT(:,3), '-b.'); 
plot(hax(6), t,   CMD(:,3)-PLANT(:,3),'-k.');
ylabel(hax(5),'Z (mm)');    title(hax(5),'Z     (mm)', 'FontWeight', 'Bold');
ylabel(hax(6),'Z err(mm)'); title(hax(6),'Z err (mm)', 'FontWeight', 'Bold');

arrayfun(@(h)(grid(h,'on')),hax)
arrayfun(@(h)(xlabel(h,'time (secs)')),hax)
arrayfun(@(h)(axis(h,'tight')),hax)

linkprop(hax,{'XLim'});

%--------------------------------------------------------------------------
% 3D error of path   
figure;

N         = 3;
err_max   = max(err_norm);
err_edges = linspace(0, err_max, N+1);
err_colors = jet(N);

clear hax
hax(1) = subplot(2,2,1); title('X err (mm)', 'FontWeight', 'Bold');
hax(2) = subplot(2,2,2); title('Y err (mm)', 'FontWeight', 'Bold');
hax(3) = subplot(2,2,3); title('Z err (mm)', 'FontWeight', 'Bold');
hax(4) = subplot(2,2,4); title('R err (mm)', 'FontWeight', 'Bold');

for kk=1:4
   set(hax(kk),'CLim',[0, err_max], 'Color', 'black');
   hold(hax(kk),'on');
   colormap(hax(kk),err_colors); colorbar(hax(kk))
   xlabel(hax(kk),'X (mm)')
   ylabel(hax(kk),'Y (mm)')
end

for kk=1:N
    % X
    subplot(2,2,1);
    X_err = abs(CMD(:,1) - PLANT(:,1));
    inds = (X_err >= err_edges(kk)) & (X_err < err_edges(kk+1)); 
    
    plot3( CMD(inds,1),  CMD(inds,2), X_err(inds), '.', ...
           'MarkerEdgeColor', err_colors(kk,:));
    % Y
    subplot(2,2,2);
    Y_err = abs(CMD(:,2) - PLANT(:,2));
    inds = (Y_err >= err_edges(kk)) & (Y_err < err_edges(kk+1)); 
    
    plot3( CMD(inds,1),  CMD(inds,2), Y_err(inds), '.', ...
           'MarkerEdgeColor', err_colors(kk,:));
    % Z
    subplot(2,2,3);
    Z_err = abs(CMD(:,3) - PLANT(:,3));
    inds = (Z_err >= err_edges(kk)) & (Z_err < err_edges(kk+1)); 
    
    plot3( CMD(inds,1),  CMD(inds,2), Z_err(inds), '.', ...
           'MarkerEdgeColor', err_colors(kk,:));
    
    % R
    subplot(2,2,4);
    inds = (err_norm >= err_edges(kk)) & (err_norm < err_edges(kk+1)); 
    
    plot3( CMD(inds,1),  CMD(inds,2), err_norm(inds), '.', ...
           'MarkerEdgeColor', err_colors(kk,:));
end
axis(hax,'tight')
linkprop(hax,{'CameraPosition','CameraUpVector','XLim','YLim'});
%--------------------------------------------------------------------------
% 3D path comparison
figure
    plot3(   CMD(:,1),   CMD(:,2),   CMD(:,3), 'r.');  hold on
    plot3( PLANT(:,1), PLANT(:,2), PLANT(:,3), '-b');

    axis tight; grid on; 
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    legend({'TARGET','delivered'})
%--------------------------------------------------------------------------    
% 2D path comparison    
figure
    plot(   CMD(:,1),    CMD(:,2), 'r.');    hold on
    plot( PLANT(:,1),  PLANT(:,2), '-b.');

    axis tight; grid on; 
    xlabel('X (mm)'); ylabel('Y (mm)');
    legend({'TARGET','delivered'})

%% and we're done
fprintf('\n ... we are finished here <%s>', mfilename);
