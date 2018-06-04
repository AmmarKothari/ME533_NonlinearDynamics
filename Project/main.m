% addpath(fullfile('../../', 'ROB541_GeometricMecahnics','Random', 'GeoOps2D'));


% Setup
f = figure(1);
clf(f);
ax = axes(f);

% Configuration
config1()

% Draw Gripper
g.draw(ax);
obj.draw(ax);
axis equal

% Draw Contact Points
hold on
plot(ax, contact_points(:,1), contact_points(:,2), 'o')
hold off

% Trajectory
% solve for angles to touch end to contact points
% minimize squared error
start_alphas = g.get_alphas();
end_pts = g.endPoints();
alphas = g.invKin(contact_points);

% Joint Space -- Linear Interp
alpha_path = linearAlphaPath(start_alphas, alphas, 100);

% Work Space
xy_path = [];
for i = 1:length(alpha_path)
    g_temp = g.calc_poses(alpha_path(i,:));
    xy_path = [xy_path; g_temp.endPoints()];
end

% Plot Workspace
hold on; plot(xy_path(:,1), xy_path(:,2), 'rx'); hold off;






