function  bh_2dof_plot_XY_given_theta( theta_1_mat_degs, ...
                                  theta_2_mat_degs, ...
                                  X_mat,       ...
                                  Y_mat, ...
                                  a_cmax)

                              
xlab_str = '\theta_1 (degs)';
ylab_str = '\theta_2 (degs)';

figure;
hax(1) = subplot(1,2,1);
   contourf(theta_1_mat_degs, theta_2_mat_degs, X_mat); 
      caxis(hax(1), [0, a_cmax]);
      colormap(gca,'jet'); colorbar
      xlabel(xlab_str, 'Interpreter', 'tex');
      ylabel(ylab_str, 'Interpreter', 'tex');
      title(hax(1), 'X_E', 'Interpreter', 'tex')
hax(2) = subplot(1,2,2);
   contourf(theta_1_mat_degs, theta_2_mat_degs, Y_mat); 
      caxis(hax(1), [0, a_cmax]);
      colormap(gca,'jet'); colorbar
      xlabel(xlab_str, 'Interpreter', 'tex');
      ylabel(ylab_str, 'Interpreter', 'tex');
      title(hax(2), 'Y_E', 'Interpreter', 'tex')

end

