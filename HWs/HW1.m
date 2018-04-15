%% Problem 1
% h=.01;
% 
% [X, Y]=meshgrid(-1:h:1,-1:h:1);      % the grid is now rectangular
% f = -10*X./Y;
% contour(X, Y, f, -10:1:10);  
% hold on                         
% dx=ones(size(X));
% quiver(X, Y, dx, f)
% axis tight
% hold off


% Plotting isoclines
grid_step = 0.1;
grid_max = 1;
grid_min = -1;
[X, Y] = meshgrid(grid_min:grid_step:grid_max, grid_min:grid_step:grid_max);

isoc_step = 0.1;
% f = @(x1, alpha) -10*x1/alpha;
f = @(x1, alpha) -10*x1/(10+alpha);
figure(1);
cla
hold on;
for alpha = [logspace(-2,0,10), logspace(0,2,20)]
    for i = 0:1
        alpha_s = alpha*(-1)^i;
        x1 = grid_min:isoc_step:grid_max;
        x2 = [];
        for x = x1 
            x2 = [x2, f(x, alpha_s)];
        end
        plot(x1, x2, '-x')
    end
end
xlim([grid_min, grid_max])
ylim([grid_min, grid_max])
hold off;