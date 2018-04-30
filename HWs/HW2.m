%% Problem 1 (Excercise 3.1)

% x = (x1^2 + x2^2
step = 1e-2;
n1_pos = @(X,X1) abs(sqrt(X^2 - X1^2));
n1_neg = @(X,X1) -abs(sqrt(X^2 - X1^2));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n1_pos(x,x1), n1_neg(x,x1)];
end

figure(1)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%%
step = 1e-2;
n2_pos = @(X,X1) abs(sqrt((X^2 - X1^2)/5));
n2_neg = @(X,X1) -abs(sqrt((X^2 - X1^2)/5));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n2_pos(x,x1), n2_neg(x,x1)];
end

figure(2)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%%
step = 1e-2;
n3_pos = @(X,X1) abs(X - abs(X1));
n3_neg = @(X,X1) -abs(X - abs(X1));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n3_pos(x,x1), n3_neg(x,x1)];
end

figure(3)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%%
step = 1e-2;
n4_pos = @(X,X1) max(X, abs(X1));
n4_neg = @(X,X1) -max(X, abs(X1));
vals = [];
x = 1; % unit ball
for x1 = -1:step:1
    vals = [vals; n4_pos(x,x1), n4_neg(x,x1)];
end

figure(4)
plot(-1:step:1, vals(:,1), 'r-', -1:step:1, vals(:,2), 'b-')
axis equal

%% Problem 2
% (i)
xs = -1e0:1e-5:1e0;
x1 = 0*xs;
x2 = 0*xs;
V = 0*xs;
V_dot = 0*xs;
i = 1;
for x = xs
    x1(i) = x^3;
    x2(i) = sin(x)^4;
    V(i) = x^2;
    V_dot(i) = -2*x^4 + 2*x*sin(x)^4;
    i = i+1;
end
% plot(xs, x1, 'rx-', xs, x2, 'bo-', xs, -x1+x2, 'g-')
plot(xs, V, 'r-', xs, V_dot, 'b-')

f = @(x) -x^3 + sin(x)^4;
fzero(f, 1)

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
