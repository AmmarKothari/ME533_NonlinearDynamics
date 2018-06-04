% Setup
f = figure(1);
clf(f);
ax = axes(f);


rev_axis = [0,0,1];
rev_axis_neg = [0,0,-1];

base = link_gripper('base', false, rev_axis, [0.5,0,0], 'g', [0,0,0]);
left_proximal = link_gripper('left_prox', 'base', rev_axis_neg, [2,0,pi/4], 'k', [1,0,0]);
left_distal = link_gripper('left_dist', 'left_prox', rev_axis_neg, [2,0,-pi/8], 'y', [1,0,0]);

g = gripper([base, left_proximal, left_distal], 1);
syms theta1 theta2
g = g.calc_poses([0, theta1, theta2]);

% Draw Gripper
% g.draw(ax);
% axis equal
