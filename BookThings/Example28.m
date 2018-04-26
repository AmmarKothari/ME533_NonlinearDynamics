syms x1 x2

f1 = @(x1,x2) 4*x1*x2^2;
f2 = @(x1,x2) 4*x1^2*x2;

df1_dx1 = diff(f1,x1);
df2_dx2 = diff(f2,x2);

df1_dx1 + df2_dx2

[t,y] = ode45(f1, [0,1], -1)

plot(t,y)