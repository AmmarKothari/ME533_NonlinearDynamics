% % % % % % % % % % % % % % % % % % 
% STOMP %
% % % % % % % % % % % % % % % % % % 
% solve for angles to touch end to contact points
% minimize squared error
path_fn = 'STOMP_Path.mat';
% if isfile(path_fn)
%     load(path_fn)
if true
    load(path_fn)
else
    end_pts = g.endPoints();
    alphas = g.invKin(contact_points);
    start_alphas = g.get_alphas();
    P = Planner(g);
    M = Map();
    M = M.addObject(obj);
    M = M.limits();
    noise_map = M.noiseCostMap(10, OBJ_NOISE);
    dist_map  = M.addDistMap(contact_points);
    blur_map  = M.blurCostMap(BLUR_FILTER_SIZE, dist_map);
    P = P.setCostMap(noise_map + blur_map, M.x_range, M.y_range);
    alpha_path = P.linearAlphaPath(start_alphas, alphas, TRAJ_STEPS);
    % 5 inputs
    q_cost = P.trajQCost(alpha_path);
    ep_cost = P.endPointCost(alpha_path, contact_points);
    eval_q = @(q) P.QCost_ind(q);
    eval_path = @(path)  sum(P.trajQCost(path)) + P.endPointCost(alpha_path, contact_points);
    S = STOMP(Q_INPUTS, TRAJ_STEPS, TRAJ_DT, eval_q, eval_path, NOISE_COOLING, g.jacobian_func(), @P.xy_path);
    stomp_path = S.optimizeTraj(STOMP_STOP_COND, alpha_path, TRAJ_NOISE, NOISEY_TRAJS);
    
    % 4 inputs -- no wrist movement
%     q_cost = P.trajQCost(alpha_path(:,2:Q_INPUTS));
%     ep_cost = P.endPointCost(alpha_path(:,2:Q_INPUTS), contact_points);
%     eval_q = @(q) P.QCost_ind(q);
%     eval_path = @(path)  sum(P.trajQCost(path)) + P.endPointCost(alpha_path(:,2:Q_INPUTS), contact_points);
%     S = STOMP(Q_INPUTS, TRAJ_STEPS, TRAJ_DT, eval_q, eval_path, NOISE_COOLING, g.jacobian_func(), @P.xy_path);
%     stomp_path = S.optimizeTraj(STOMP_STOP_COND, alpha_path(:,2:Q_INPUTS), TRAJ_NOISE, NOISEY_TRAJS);

    save(path_fn, 'stomp_path')
end