%% Lecture 2
test = @(t, x0) (x0*exp(-t))/(1-x0+x0*exp(-t));
t_total = 10;
x0 = 1.5;
x = [];
for t = 0:0.01:t_total
    x = [x; test(t, x0)];
    
end
figure()
plot(x, 'bx-')

% try with ode45


%% Lecture 3