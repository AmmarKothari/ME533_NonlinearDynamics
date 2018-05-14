%% HW3

%% Problem 1 (7.7)
xdd_plant = @(alpha1, alpha2, x, v) [x(2), -alpha1*abs(x(1))*x(2)^2-alpha2*x(1)^3*cos(2*x(1))+v];

syms x1 x2 v

ts = 0:1e-3:pi/100;
xs = [];
alpha1s = [-1, 0, 1, 2*(rand(1)-0.5)];
alpha2s = [-1, 2, 5, 6*(rand(1)-0.5)];
i1 = 0;

%trajectory
x_des = sin(ts)';
xd_des = diff(x_des); xd_des = [xd_des; xd_des(end)];
xdd_des = diff(xd_des); xdd_des = [xdd_des; xdd_des(end)];
trajectory = [x_des, xd_des, xdd_des];
subplot(3,2,1)
plot(ts, x_des, 'rx')
subplot(3,2,2)
plot(ts, xd_des, 'bo')

% contoller constants
% v = 5*ud + u
eta = 0.1;
lambda = 1/100;
k_func = @(x) -abs(x(1))*x(2)^2 - 3*x(1)^3*cos(2*x(1)) + eta;
s_func = @(x,x_des) x(2)-x_des(2) + lambda*(x(1) - x_des(1));
v_func = @(x,x_des) 2*x(1)^2*cos(2*x(1)) + x_des(3) - lambda*(x(2)-x_des(2)) - k_func(x)*sign(s_func(x,x_des));
vs = [];

% test different plants
for alpha1 = alpha1s
    i1 = i1 + 1;
    i2 = 0;
    for alpha2 = alpha2s
        xdd = @(x, v) xdd_plant(alpha1, alpha2, x, v);
        fprintf('Alpha 1: %0.4f, Alpha2: %0.4f \n', alpha1, alpha2)
        plant = xdd([x1, x2], v);
        fprintf('Actual Plant: %s \n', plant(2))
        
        i2 = i2 + 1;
        x_temp = [];
        v_temp = [];
        it = 0;
        t_last = 0;
        x = trajectory(1,1:2);
        for t = ts
            it= it+1;
            v = v_func(x, trajectory(it,:));
            v_temp = [v_temp; v];
            dt = t-t_last;
            x_new = x + dt*xdd(x, v);
            x_temp = [x_temp; x_new];
            x = x_new;
        end
        xs = cat(3, xs, x_temp);
        vs = cat(3, vs, v_temp);
        
        % position
        subplot(3,2,3)
%         hold on
        plot(ts, x_temp(:,1), 'rx')
        title('Position')
        
        % velocity
        subplot(3,2,4)
%         hold off
        plot(ts, x_temp(:,2), 'bo')
        title('Velocity')
        
        % control input
        subplot(3,2,5)
        plot(ts, v_temp, 'kx-')
        title('Control Input')
        
    end
end
