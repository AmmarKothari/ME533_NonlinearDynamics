clf(figure(1));
alpha1 = @(t) 1+abs(sin(t)); % 1 <= alpha <=2

f = @(t,x) -alpha1(t)*x(2)^2*cos(3*x(1));
plant = @(t,x,v) f(t,x) + v;

alpha1_hat = @(t) 1.5;
f_hat = @(t,x) -alpha1_hat(t)*x(2)^2*cos(3*x(1));

alpha1_limit = @(t) 0.5;
F = @(t,x) alpha1_hat(t)*x(2)^2*abs(cos(3*x(1)));

dt = 1e-3;
t_end = 2;
ts = 0:dt:t_end;

% Controller
lambda = 20;
eta = 0.1;
phi = 0.1;
u_max = 5;
u_min = -5;

s_func = @(x_err) x_err(2)+lambda*x_err(1);

xs = -100*(ones(length(ts),3));
vs = -100*(ones(length(ts),1));
ss = -100*(ones(length(ts),1));
% % Plant only plot
% x = [1,-1];
% v = 0;
% for it = 1:length(ts)
%     xdd = plant(ts(it), x, v);
%     x = x + dt*[x(2), xdd];
%     
%     xs(it, :) = [x, xdd];
% %     plot(ts(1:it), xs(1:it,1), 'rx')
% %     drawnow()
% end

% xdd_des = [1e-3*0:dt:1, zeros(length((1+dt):dt:t_end), 1)']';
x_des = sin(pi/2*ts');
xd_des = diff(x_des)/dt; xd_des = [xd_des(1); xd_des];
xdd_des = diff(xd_des)/dt; xdd_des = [xdd_des(1); xdd_des];
traj = [x_des, xd_des, xdd_des];
x = traj(1, 1:2);

for it = 1:length(ts)
    % determine control input
    x_err = x - traj(it,1:2); % don't care about error in acceleration because that is an input!
    u_hat = -f_hat(ts(it),x) + traj(it,3) - lambda*x_err(2);
    k = F(ts(it),x) + eta;
    s = s_func(x_err);
%     v = u_hat - k*sign(s);
    v = u_hat - k*min(1, max(-1, s/phi));
%     v = min(u_min, max(u_max, v_raw));
    % plot(ts(1:it), x_des(1:it,1), 'rx')
    
    % run dynamics forward
    xdd = plant(ts(it), x, v);
    x = x + dt*[x(2), xdd];
    
    xs(it, :) = [x, xdd];
    vs(it) = v;
    ss(it) = s;
    
    
end
    
% plotting
% plot(ts, xs(:,1), 'rx')
% plot(ts, vs(:,1), 'bx-')


% position
subplot(3,2,1)
hold on
plot(ts, traj(:,1), 'rx')
plot(ts, xs(:,1), 'bo')
title('Position')
legend('Ref', 'Actual')

% position error
subplot(3,2,3)
hold on
plot(ts, xs(:,1) - traj(:,1), 'kx-')
title('Position Tracking Error')

% velocity
subplot(3,2,2)
hold on
plot(ts, traj(:,2), 'rx')
plot(ts, xs(:,2), 'bo')
title('Velocity')
legend('Ref', 'Actual')

% velocity error
subplot(3,2,4)
hold on
plot(ts, xs(:,2) - traj(:,2), 'kx-')
title('Position Tracking Error')

% control input
subplot(3,2,5)
plot(ts, vs, 'kx-')
title('Control Input')

% s plot
subplot(3,2,6)
plot(ts, ss, 'bo-')
title('s plot')
% fprintf('Switch Count: %d \n', switch_count)
