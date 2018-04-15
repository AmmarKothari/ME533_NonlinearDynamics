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


%% Problem 2
r_dot = @(r,t) r*(r^2-1) *sin(1/(r^2-1));
X = -2:0.01:2;
Y = [];
for x=X
    Y = [Y, r_dot(x)];
end
% plot(X,Y)
for r_0 = logspace(-3,0.1,10)
    r_0 = 0.9999999999999;
    r_all = [];
    r = r_0;
    dt  = 0.1;
    T = 1:dt:105;
    for t = T
        r = r + r_dot(r,t)*dt;
        r_all = [r_all, r];
    end
    plot(T,r_all);
    fprintf('Starting Radius: %s, Final Value: %s \n',r_0, r_all(end))
end

