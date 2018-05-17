%% HW 3 attempt 2

%% Problem 1 (7.7)
clf(figure(1));
alpha1 = @(t) sin(t);
alpha2 = @(t) cos(t);

f = @(t,x) -alpha1(t)*abs(x(1))*x(2)^2 - alpha2(t)*x(1)^3*cos(2*x(1));
plant = @(t,x,v) -alpha1(t)*abs(x(1))*x(2)^2 - alpha2(t)*x(1)^3*cos(2*x(1)) + v;

alpha1_hat = @(t) 0;
alpha2_hat = @(t) 2;
f_hat = @(t,x) -alpha1_hat(t)*abs(x(1))*x(2)^2 - alpha2_hat(t)*x(1)^3*cos(2*x(1));

alpha1_limit = @(t) 1;
alpha2_limit = @(t) 3;
F = @(t,x) -alpha1_hat(t)*abs(x(1))*x(2)^2 - alpha2_hat(t)*x(1)^3*cos(2*x(1));

dt = 1e-2;
t_end = 10;
ts = 0:dt:t_end;

% Controller
lambda = 100;
eta = 1;

s_func = @(x_err) x_err(2)+lambda*x_err(1);

xs = -100*(ones(length(ts),3));
vs = -100*(ones(length(ts),1));
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
xdd_des = sin(pi/2*ts');
xd_des = cumsum(dt*xdd_des);
x_des = cumsum(dt*xd_des);
traj = [x_des, xd_des, xdd_des];
x = traj(1, 1:2);

for it = 1:length(ts)
    % determine control input
    x_err = x - traj(it,1:2); % don't care about error in acceleration because that is an input!
    u_hat = -f_hat(ts(it),x) + traj(it,3) - lambda*x_err(2);
    k = F(ts(it),x) + eta;
    s = s_func(x_err);
    v = u_hat - k*sign(s);
    % plot(ts(1:it), x_des(1:it,1), 'rx')
    
    % run dynamics forward
    xdd = plant(ts(it), x, v);
    x = x + dt*[x(2), xdd];
    
    xs(it, :) = [x, xdd];
    vs(it) = v;
    
    
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
% subplot(3,2,6)
% plot(ts, s_temp, 'kx-')
% title('S Plot')


%% Problem 2 7.10
clf(figure(1));
alpha1 = @(t) -1;
alpha2 = @(t) -2;
b = @(t) 1;
f = @(t,x) -alpha1(t)*x(3)^2 - alpha2(t)*x(2)^5*sin(4*x(1));
plant = @(t,x,u) -alpha1(t)*x(3)^2 - alpha2(t)*x(2)^5*sin(4*x(1)) + b(t)*u;

m = 3; % order of plant

alpha1_hat = @(t) 0;
alpha2_hat = @(t) 0;
b_hat = @(t) sqrt(1*4);
f_hat = @(t,x) -alpha1_hat(t)*x(3)^2 - alpha2_hat(t)*x(2)^5*sin(4*x(1));

alpha1_limit = @(t) 2;
alpha2_limit = @(t) 4;
F = @(t,x) alpha1_hat(t)*x(3)^2 + alpha2_hat(t)*x(2)^5*sin(4*x(1));

dt = 1e-3;
t_end = 5;
ts = 0:dt:t_end;

% Controller
B = 2;
lambda = 100;
eta = 0.1;
phi=1;

s_func = @(x_err) x_err(3)+2*lambda*x_err(2)+lambda^2*x_err(1);

xs = -100*(ones(length(ts),m+1));
vs = -100*(ones(length(ts),1));
ss = -100*(ones(length(ts),1));
% % Plant only plot
% x = [1,-1, 0];
% v = 0;
% for it = 1:length(ts)
%     xdd = plant(ts(it), x, v);
%     x = x + dt*[x(2:m), xdd];
%     
%     xs(it, :) = [x, xdd];
%     plot(ts(1:it), xs(1:it,1), 'rx')
%     drawnow()
% end

% xdd_des = [1e-3*0:dt:1, zeros(length((1+dt):dt:t_end), 1)']';
xddd_des = sin(pi/2*ts');
xdd_des = cumsum(dt*xddd_des);
xd_des = cumsum(dt*xdd_des);
x_des = cumsum(dt*xd_des);
traj = [x_des, xd_des, xdd_des, xddd_des];
x = traj(1, 1:m);
switch_count = 0;

sigmoid = @(X) 2/(1+exp(-X))-1;
for it = 1:length(ts)
    % determine control input
    x_err = x - traj(it,1:m); % don't care about error in acceleration because that is an input!
    u_hat = -f_hat(ts(it),x) + traj(it,end) - 2*lambda*x_err(3) - lambda^2*x_err(2);
    k = B*(F(ts(it),x) + eta) + abs(B-1)*abs(u_hat);
    s = s_func(x_err);
    v = u_hat - k*sign(s);
%     v = u_hat - k*min(1, max(-1, s/phi));
%     v = u_hat - k*sigmoid(s);
    % plot(ts(1:it), x_des(1:it,1), 'rx')
    
    % run dynamics forward
    xddd = plant(ts(it), x, v);
    x = x + dt*[x(2:m), xddd];
    
    xs(it, :) = [x, xddd];
    vs(it) = v;
    if it>1 && sign(s) ~= sign(ss(it-1))
        switch_count = switch_count + 1;
    end
    ss(it) = s;
    
    
    
end
    
% plotting
% plot(ts, xs(:,1), 'rx')
% plot(ts, vs(:,1), 'bx-')


% position
subplot(3,3,1)
hold on
plot(ts, traj(:,1), 'rx')
plot(ts, xs(:,1), 'bo')
title('Position')
legend('Ref', 'Actual')

% position error
subplot(3,3,4)
hold on
plot(ts, xs(:,1) - traj(:,1), 'kx-')
title('Position Tracking Error')

% velocity
subplot(3,3,2)
hold on
plot(ts, traj(:,2), 'rx')
plot(ts, xs(:,2), 'bo')
title('Velocity')

% velocity error
subplot(3,3,5)
hold on
plot(ts, xs(:,2) - traj(:,2), 'kx-')
title('Velocity Tracking Error')

% Acceleration
subplot(3,3,3)
hold on
plot(ts, traj(:,3), 'rx')
plot(ts, xs(:,3), 'bo')
title('Acceleration')

% Acceleration error
subplot(3,3,6)
hold on
plot(ts, xs(:,3) - traj(:,3), 'kx-')
title('Acceleration Tracking Error')

% control input
subplot(3,3,7)
plot(ts, vs, 'kx-')
title('Control Input')

% s plot
subplot(3,3,8)
plot(ts, ss, 'bo-')
title('s plot')
fprintf('Switch Count: %d \n', switch_count)

%% Problem 3 8.3???
ap1=0.1; ap2=-4; bp=2;
bv=1/bp; av1=1/bv; av2=ap1/bv; av3=ap2/bv;

plant = @(t,y,v) 1/av1*(av2*y(2) - av3*y(1) + v);
m = 2;


dt = 1e-3;
t_end = 5;
ts = 0:dt:t_end;

ys = -100*(ones(length(ts),m+1));
vs = -100*(ones(length(ts),1));
ss = -100*(ones(length(ts),1));

% % Plant only plot
% y = [1,-1];
% v = 0;
% for it = 1:length(ts)
%     ydd = plant(ts(it), y, v);
%     y = y + dt*[y(2:m), ydd];
%     
%     ys(it, :) = [y, ydd];
%     plot(ts(1:it), ys(1:it,1), 'rx')
%     drawnow()
% end

% Controller
lambda = 100;
k = 10; % is this ok?
am1 = 0.5; am2 = 1; am3 = -1;
am = [am1; am2; am3];
ams = -100*(ones(length(ts),length(am)));
s_func = @(x_err) x_err(2)+lambda*x_err(1);
P = [1, 0.1, 0; 0.1, 1, 0; 0, 0, 1];

% ydd_des = [1e-3*0:dt:1, zeros(length((1+dt):dt:t_end), 1)']';
ydd_des = sin(pi/2*ts');
yd_des = cumsum(dt*ydd_des);
y_des = cumsum(dt*yd_des);
traj = [y_des, yd_des, ydd_des];
y = traj(1, 1:m);
switch_count = 0;

sigmoid = @(X) 2/(1+exp(-X))-1;
for it = 1:length(ts)
    % determine control input
    y_err = y - traj(it,1:m); % don't care about error in acceleration because that is an input!
    s = s_func(y_err);
    ydd_r = traj(it,m) - lambda*y_err(2);
    Y = [ydd_r, y(end:-1:1)];
    v = Y*am - k*s;
    ad_hat = -P*Y'*s;
%     v = u_hat - k*min(1, max(-1, s/phi));
%     v = u_hat - k*sigmoid(s);
    % plot(ts(1:it), x_des(1:it,1), 'rx')
    
    % run dynamics forward
    ydd = plant(ts(it), y, v);
    y = y + dt*[y(2:m), ydd];
    
    % update parameters of model
    am = am + ad_hat*dt;
    
    ys(it, :) = [y, ydd];
    vs(it) = v;
    ams(it, :) = am';
    if it>1 && sign(s) ~= sign(ss(it-1))
        switch_count = switch_count + 1;
    end
    ss(it) = s;
    
    
    
end

% position
subplot(3,3,1)
hold on
plot(ts, traj(:,1), 'rx')
plot(ts, ys(:,1), 'bo')
title('Position')
legend('Ref', 'Actual')

% position error
subplot(3,3,4)
hold on
plot(ts, ys(:,1) - traj(:,1), 'kx-')
title('Position Tracking Error')

% velocity
subplot(3,3,2)
hold on
plot(ts, traj(:,2), 'rx')
plot(ts, ys(:,2), 'bo')
title('Velocity')

% velocity error
subplot(3,3,5)
hold on
plot(ts, ys(:,2) - traj(:,2), 'kx-')
title('Velocity Tracking Error')

% Acceleration
subplot(3,3,3)
hold on
plot(ts, traj(:,3), 'rx')
plot(ts, ys(:,3), 'bo')
title('Acceleration')

% Acceleration error
subplot(3,3,6)
hold on
plot(ts, ys(:,3) - traj(:,3), 'kx-')
title('Acceleration Tracking Error')

% control input
subplot(3,3,7)
plot(ts, vs, 'kx-')
title('Control Input')

% s plot
subplot(3,3,8)
plot(ts, ss, 'bo-')
title('s plot')
fprintf('Switch Count: %d \n', switch_count)

% parameter plot
figure(2)
ap1_m = ams(:,2)./ams(:,1);
ap2_m = ams(:,3)./ams(:,1);
subplot(2,1,1)
plot(ts, ap1_m,'bx', ts, ones(length(ts), 1)*ap1, 'r-')
ylim([-2*ap1,2*ap1])
title('a_{p1}')

subplot(2,1,2)
plot(ts, ap2_m, 'bx', ts, ones(length(ts), 1)*ap2, 'r-')
ylim([-2*abs(ap2),2*abs(ap2)])
title('a_{p2}')
