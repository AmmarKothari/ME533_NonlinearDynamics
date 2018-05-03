%% Problem 1 (Excercise 3.1)

% x = (x1^2 + x2^2)
step = 1e-2;
vals = [];
x = 1; % unit ball

n1_pos = @(X,X1) abs(sqrt(X^2 - X1^2));
n1_neg = @(X,X1) -abs(sqrt(X^2 - X1^2));
for x1 = -1:step:1
    vals = [vals; n1_pos(x,x1), n1_neg(x,x1)];
end

subplot(2,2,1)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%
step = 1e-2;
n2_pos = @(X,X1) abs(sqrt((X^2 - X1^2)/5));
n2_neg = @(X,X1) -abs(sqrt((X^2 - X1^2)/5));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n2_pos(x,x1), n2_neg(x,x1)];
end

subplot(2,2,2)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%
step = 1e-2;
n3_pos = @(X,X1) abs(X - abs(X1));
n3_neg = @(X,X1) -abs(X - abs(X1));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n3_pos(x,x1), n3_neg(x,x1)];
end

subplot(2,2,3)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%
step = 1e-2;
n4_pos = @(X,X1) max(X, abs(X1));
n4_neg = @(X,X1) -max(X, abs(X1));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n4_pos(x,x1), n4_neg(x,x1)];
end

subplot(2,2,4)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%% Problem 2
%% (i)
ts = 0:1e-1:5;
x_dot =@(t,x) -x^3 + sin(x)^4;
[t, y] = ode45(x_dot, ts, -1);
subplot(3,1,1)
plot(t, y(:,1), 'ro-', t(1:end-1), diff(y), 'bx-')
legend('x', 'x-dot')

xs = -1e0:1e-5:1e0;
V = 0*xs;
V_dot = 0*xs;
i = 1;
for x = xs
    V(i) = x^2;
    V_dot(i) = -2*x^4 + 2*x*sin(x)^4;
    i = i+1;
end
% plot(xs, x1, 'rx-', xs, x2, 'bo-', xs, -x1+x2, 'g-')
subplot(3,1,2)
plot(xs, V, 'r-')
title('V')
subplot(3,1,3)
plot(xs, V_dot, 'b-')
title('V-Dot')
%% (ii)
ts = 0:1e-1:5;
x_dot =@(t,x) (5-x)^5;
[t, y] = ode45(x_dot, ts, -1);
subplot(3,1,1)
plot(t, y(:,1), 'ro-', t(1:end-1), diff(y), 'bx-')
legend('x', 'x-dot')

xs = -1e0:1e-5:1e0;
V = 0*xs;
V_dot = 0*xs;
i = 1;
for x = xs
    V(i) = x^2;
    V_dot(i) = -2*x^6;
    i = i+1;
end
% plot(xs, x1, 'rx-', xs, x2, 'bo-', xs, -x1+x2, 'g-')
subplot(3,1,2)
plot(xs, V, 'r-')
title('V')
subplot(3,1,3)
plot(xs, V_dot, 'b-')
title('V-Dot')

%% (iii)
ts = 0:1e-1:20;
x_dd =@(t,x) [x(2); -x(2)^5-x(1)^7 + x(1)^2*sin(x(1))^8*cos(3*x(1))^2];
[t, y] = ode45(x_dd, ts, [-1; 0]);
subplot(3,1,1)
plot(t, y(:,1), 'ro-', t, y(:,2), 'bx-', t(1:end-1), diff(y(:,2)), 'g^-')
legend('x', 'xd', 'xdd')

xs = -1e0:1e-1:1e0;
xds = -1e0:1e-1:1e0;
V = zeros(length(xs), length(xds));
V_dot = 0*V;
i_d = 1;
for x = xs
    i_dd = 1;
    for xd = xds
        V(i_d, i_dd) = 1/2*xd^2 + integral(@(y) y.^7-y.^2.*sin(y).^8.*cos(3*y).^2, 0, x);
        V_dot(i_d,i_dd) = -xd^6;
        i_dd = i_dd+1;
    end
    i_d = i_d+1;
end
subplot(3,1,2)
surf(xs, xds, V)
title('V')
subplot(3,1,3)
surf(xs, xds, V_dot)
title('V-Dot')
%% (iv)
ts = 0:1e-1:20;
x_dd =@(t,x) [x(2); -(x(1)-1).^4*x(2).^7-x(1).^5-x(1)^3*sin(x(1))^3];
[t, y] = ode45(x_dd, ts, [0; -1e-1]);
subplot(3,1,1)
plot(t, y(:,1), 'ro-', t, y(:,2), 'bx-', t(1:end-1), diff(y(:,2)), 'g^-')
legend('x', 'xd', 'xdd')

xs = -1e0:1e-1:1e0;
xds = -1e0:1e-1:1e0;
V = zeros(length(xs), length(xds));
V_dot = 0*V;
i_d = 1;
for x = xs
    i_dd = 1;
    for xd = xds
        V(i_d, i_dd) = 1/2*xd^2 + integral(@(y) y.^5-y.^3.*sin(y).^3, 0, x);
        V_dot(i_d,i_dd) = -xd^8*(x-1)^4;
        i_dd = i_dd+1;
    end
    i_d = i_d+1;
end
subplot(3,1,2)
surf(xs, xds, V)
title('V')
subplot(3,1,3)
surf(xs, xds, V_dot)
title('V-Dot')
%% (v)
ts = 0:1e-1:20;
x_dd =@(t,x) [x(2); -(x(1)-1).^2.*x(2).^7-x(1)+sin(pi.*x(1)/2)];
[t, y] = ode45(x_dd, ts, [1; 0]);
subplot(3,1,1)
plot(t, y(:,1), 'ro-', t, y(:,2), 'bx-', t(1:end-1), diff(y(:,2)), 'g^-')
legend('x', 'xd', 'xdd')

a = 1e-1; 1/2*a^2 + cos(pi*a/2)*2/pi-2/pi

syms x xd
f = [xd; -(x-1)^2*xd^7 - x + sin(pi*x/2)];
A = jacobian(f);
F = A + transpose(A);
f = @(x, xd) [                                     0, (2 - 2.*x).*xd.^7 + (pi.*cos((x.*pi)/2))/2;
             (2 - 2.*x).*xd.^7 + (pi.*cos((x.*pi)/2))/2,                    -14.*xd.^6*(x - 1).^2];
xs = -1e0:1e-1:1e0;
xds = -1e0:1e-1:1e0;
for x = xs
    i_dd = 1;
    for xd = xds
        f(xs, xds)
    end
end
%%
xs = -1e0:1e-1:1e0;
xds = -1e0:1e-1:1e0;
V = zeros(length(xs), length(xds));
V_dot = 0*V;
i_d = 1;
for x = xs
    i_dd = 1;
    for xd = xds
        V(i_d, i_dd) = 1/2*xd^2 + integral(@(y) x(1)-sin(pi.*x(1)/2), 0, x);
        V_dot(i_d,i_dd) = -xd^8*(x-1)^4;
        i_dd = i_dd+1;
    end
    i_d = i_d+1;
end
subplot(3,1,2)
surf(xs, xds, V)
title('V')
subplot(3,1,3)
surf(xs, xds, V_dot)
title('V-Dot')
%% Problem 3
dt = 1e-1;
t_total = 5;
figure(1); cla; hold on;
% for a = logspace(-3,1)
a = 1e-2;
b = 1e-0; c = 1;
v_dot = @(v) -2*a*abs(v)*v - b*v + c;
v_func = @(dt, x) v_dot(x(1));
[t, y] = ode45(v_func, 0:dt:t_total, 1);
plot(t, y(:,1), 'ro-', t(1:end-1), diff(y), 'bx-')
legend('Velocity', 'Acceleration')
% end

% figure(2); cla; hold on;
% for b = logspace(-3,0)
%     a = 1e-2;
% %     b = 1e-3;
%     c = 1e-2;
%     v_dot = @(v) -2*a*abs(v)*v - b*v + c;
%     v_func = @(dt, x) [v_dot(x(1)); 0];
%     [t, y] = ode45(v_func, 0:dt:t_total, [0; 0]);
%     plot(t, y(:,1), 'o-', t, y(:,2), 'bx-')
% end

%% Problem 4
%% a
syms t
A = [-10, exp(3*t);
    0, -2];
eig(A + transpose(A))
eigs = @(t) [(exp(6*t) + 64)^(1/2) - 12;
            - (exp(6*t) + 64)^(1/2) - 12];

%% b
syms t
A = [-1, 2*sin(t);
    0, -(t+1)];

eig(A + transpose(A))
eigs = @(t) [(4*sin(t)^2 + t^2)^(1/2) - t - 2;
             - t - (4*sin(t)^2 + t^2)^(1/2) - 2];
