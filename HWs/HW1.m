%% Problem 1
h=.01;

[X, Y]=meshgrid(-1:h:1,-1:h:1);      % the grid is now rectangular
f = -10*X./Y;
contour(X, Y, f, -10:1:10);  
hold on                         
dx=ones(size(X));
quiver(X, Y, dx, f)
axis tight
hold off