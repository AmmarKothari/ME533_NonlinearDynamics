% % % % % % % % % % % % % % % % % % % % % 
% Hand with two links with two joints each %
% % % % % % % % % % % % % % % % % % % % % 
rev_axis = [0,0,1];
rev_axis_neg = [0,0,-1];

l1 = 1;
l2 = 2;

% base = link_gripper('base', false, rev_axis, [0.5,0,0], 'g', [0,0,0]);
base = link_gripper('base', false, [0,0,0], [0.5,0,0], 'g', [0,0,0]);
left_proximal = link_gripper('left_prox', 'base', rev_axis_neg, [l1,0,pi/4], 'k', [1,0,0]);
left_distal = link_gripper('left_dist', 'left_prox', rev_axis_neg, [l2,0,-pi/8], 'y', [1,0,0]);
right_proximal = link_gripper('right_prox', 'base', rev_axis, [l1,0,0], 'b', [1,0,0]);
right_distal = link_gripper('right_dist', 'right_prox', rev_axis, [l2,0,pi/8], 'r', [1,0,0]);
g = gripper([base, left_proximal, left_distal, right_proximal, right_distal], 1);
g = g.calc_poses([0, -pi/8, 0, 0, 0]);