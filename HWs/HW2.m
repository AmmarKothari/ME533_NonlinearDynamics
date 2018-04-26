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
xs = -1e0:1e-3:1e0;
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