%% HW3

%% Problem 1 (7.7)
xdd_plant = @(alpha1, alpha2, x, u) [x(2), -alpha1*abs(x(1))*x(2)^2-alpha2*x(1)^3*cos(2*x(1))+5*u(2)+u(1)];

syms x1 x2 u1 u2

ts = 0:1e-3:2*pi;
x = [1,1];
xs = [];
alpha1s = [-1, 0, 1, 2*(rand(1)-0.5)];
alpha2s = [-1, 2, 5, 6*(rand(1)-0.5)];
i1 = 0;

%trajectory
x_des = sin(ts)';
xd_des = diff(x_des); xd_des = [xd_des; xd_des(end)];
trajectory = [x_des, xd_des];
subplot(3,2,1)
plot(ts, x_des, 'rx')
subplot(3,2,2)
plot(ts, xd_des, 'bo')

% contoller constants
eta = 0.1;
lambda = 1/100;


% test different plants
for alpha1 = alpha1s
    i1 = i1 + 1;
    i2 = 0;
    for alpha2 = alpha2s
        xdd = @(x, u) xdd_plant(alpha1, alpha2, x, u);
        fprintf('Alpha 1: %0.4f, Alpha2: %0.4f \n', alpha1, alpha2)
        plant = xdd([x1, x2], [u1, u2]);
        fprintf('Actual Plant: %s \n', plant(2))
        
        % controller
        % v = 5*ud + u
        k = @(x) -abs(x(1))*x(2)^2 - 3*x(1)^3*cos(2*x(1)) + eta;
        
        i2 = i2 + 1;
        x_temp = [];
        for t = ts
            v = 2*x(1)^3*cos(2*x(1)) + xdd_des + ;
            
            x = xdd(x, [0,1e-5]);
            x_temp = [x_temp; x];

        end
        xs = cat(3, xs, x_temp);
        subplot(3,2,3)
%         hold on
        plot(ts, x_temp(:,1), 'rx')
        subplot(3,2,4)
%         hold off
        plot(ts, x_temp(:,2), 'bo')
    end
end
