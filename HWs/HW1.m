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

% prove limit cycles
r_dot = @(r,t) r*(r^2-1) *sin(1/(r^2-1));
r_limit_cyle = @(n) sqrt(1/(n*pi) + 1);
r_dot_all = [];
figure(1); cla; hold on;
for pi_multiples = 10:10
    r_0 = r_limit_cyle(pi_multiples);
    r_all = [];
    r = r_0;
    dt  = 0.01;
    T = 0:dt:10;
    for t = T
        r_dot_all = [r_dot_all, r_dot(r,t)]; 
        r = r + r_dot_all(end)*dt;
        r_all = [r_all, r];
    end
%     plot(T,r_all);
    plot(r_all, r_dot_all, 'rx-');
    fprintf('Starting Radius: %s, Final Value: %s \n',r_0, r_all(end))
    pause(1)
end

%% stability
disturbance = 1e-3;
for pi_multiples = 1:10
    for dr = [-1, 1]
        r_dot_temp = r_dot(r_limit_cyle(pi_multiples)+dr*disturbance);
        fprintf('Multiple: %d, Disturbance: %d, r_dot: %0.20f+%fj\n', pi_multiples, dr, real(r_dot_temp), imag(r_dot_temp))
    end
end
%%
y_dot = @(y,u) (sqrt(2*u*y) - 10*y)*[1;-1];
u_all = 0:1:2;
y_all = 0:0.001:0.3;
figure(1); cla; hold on;
for u = u_all
    y_dot_all = [];
    for y = y_all
        y_dot_all = [y_dot_all; y_dot(y,u)];
    end
    plot(y_all, y_dot_all(2:2:end), 'rx-')
    plot(y_all, y_dot_all(1:2:end), 'bx-')
end
hold off







