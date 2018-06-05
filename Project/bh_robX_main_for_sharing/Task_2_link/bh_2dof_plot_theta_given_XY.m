function  bh_2dof_plot_theta_given_XY( X_mat,       ...
                                       Y_mat, ...
                                       theta_1_mat_degs, ...
                                       theta_2_mat_degs)

                              
xlab_str = 'X (m)';
ylab_str = 'Y (m)';

hfig = figure;
hax(1) = subplot(1,2,1);
   contourf(X_mat, Y_mat, theta_1_mat_degs); 
      caxis(hax(1), [-180, 180]);
      colormap(gca,'jet'); colorbar
      xlabel(xlab_str, 'Interpreter', 'tex');
      ylabel(ylab_str, 'Interpreter', 'tex');
      title(hax(1), '\theta_1', 'Interpreter', 'tex')
      axis('equal')
hax(2) = subplot(1,2,2);
   contourf(X_mat, Y_mat, theta_2_mat_degs); 
      caxis(hax(2), [-180, 180]);
      colormap(gca,'jet'); colorbar
      xlabel(xlab_str, 'Interpreter', 'tex');
      ylabel(ylab_str, 'Interpreter', 'tex');
      title(hax(2), '\theta_2', 'Interpreter', 'tex')
      axis('equal')
      
end

