% ISOCLINE PLOTS

% Consider the differential equation y'=f(x,y). 
% For fixed c, the set of all points (x,y) such that f(x,y)=c is a level set of f, typically a curve.
% At the same time, this is the set of all points such that y'=c, an isocline.
% By varying c we may find the whole spectrum of isoclines.
% Together they form a contour map that helps to understand the differential equation geometrically.
% How to plot a contour map?

% MATLAB has convenient built-in commands for this purpose:
% 'meshgrid' creates an array used for evaluating f(x,y)
% 'contour' allows us to produce an actual plot
% Type 'help meshgrid' and 'help contour' to learn more about these commands.

% Let's see how this works on the example of y'=x^2-y^2.

h=.1;                               % choose a stepsize

[X, Y]=meshgrid(-1:h:1);            % create a square array of points (x,y)

f=X.^2-Y.^2                         % evaluate f(x,y)=x^2-y^2 at points of the array

contour(X, Y, f);                   % a beautiful map appears

% The choice of constants c in this plot was done by MATLAB.
% To specify c to your liking, use the following syntax:

pause                               % hit any key to resume

c=-1:h:1;                           % create an array of constants

contour(X, Y, f, c);                % a map even more beautiful 

hold on                             % hold the current plot

pause                               % hit any key to resume

% Let's add the slope vector field to our plot.  

dx=ones(size(X));                   % create an array of increments dx=1

quiver(X, Y, dx, f)                 % slopes are added

axis tight                          % optimize space

pause

% Type 'help quiver' to learn more.   

% Consider another example: y'=-x/y.

hold off                            % reset plot settings

h=.4;

[X, Y]=meshgrid(-5:h:5,1:h:5);      % the grid is now rectangular

f=-X./Y;                             

contour(X, Y, f, -10:.2:10);  

hold on                         

pause                               

% Add the slope vector field. 

dx=ones(size(X));

quiver(X, Y, dx, f)

axis tight

hold off