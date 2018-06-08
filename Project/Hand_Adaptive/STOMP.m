classdef STOMP
    properties
       A, R, R_inv, M
       DOF
       steps, dt
       eval_state
       eval_path
       best_traj_iter % best trajectory from that iteration
       noise_cooling % reducing noise in each iteration
       noisey_trajs_xy
       vel_func
       xy_path_func
       NO_IMPROVEMENT_STOP = 10
       %
       debugSTOMP1 = false, %plot noisey trajectories
       debugSTOMP2 = false, % plot noisey trajectories in operational space
    end
    methods
        function obj = STOMP(dof, steps, dt, eval_state, eval_path, noise_cooling, vel_func,xy_path_func)
            obj.DOF = dof;
            obj.steps = steps;
            obj.eval_state = eval_state; % function to evaluate cost of a state
            obj.eval_path = eval_path; % function to evaluate cost of path
            %precompute
            obj.A = obj.computeA(steps);
            obj.R = obj.A' * obj.A;
            obj.R_inv = inv(obj.R);
            obj.M = obj.computeM(obj.R_inv, steps);
            obj.dt = dt;
            obj.noise_cooling = noise_cooling;
            obj.vel_func = vel_func;
            obj.xy_path_func = xy_path_func;
        end
        
        function cur_path = optimizeTraj(obj, stop_condition, path, noise_amp, n_trajs)
            ITER_LIMIT = 100;
            eval_path = @(p) obj.eval_path(p) + sum(sum(1/2 * p' * obj.R * p));
            Q_new = eval_path(path);
            Q_prev = Q_new * 10;
            cur_path = path;
            no_improvement_counter = 0;
            total_iters = 0;
            while (no_improvement_counter < obj.NO_IMPROVEMENT_STOP)
                total_iters = total_iters + 1;
                if total_iters > ITER_LIMIT
                    break
                end
                [noisey_trajs, noise, range] = obj.noiseyTrajs(cur_path, noise_amp, n_trajs); % create noisey trajectories
                traj_costs = obj.evalTrajs(noisey_trajs); % evaluate each point in trajectories
                traj_probs = obj.probTrajs(traj_costs); % calculate probability of each point based on cost
                delta_q = obj.deltaQ(traj_probs, noise, obj.M); % change in path
                delta_q = delta_q / 100; % I don't know.  Otherwise it was generating crazy trajectories.
                new_path = obj.adjustPath(cur_path, delta_q, range); % updated path
                Q_new = eval_path(new_path);
                Q_diff = abs(Q_new - Q_prev)/Q_prev;
                noise_amp = noise_amp * obj.noise_cooling;
                fprintf('Iter: %d, Previous Cost: %f \t New Cost: %f \n', total_iters, Q_prev, Q_new)
%                 obj.plotNoiseyTrajs(path, cat(3, cur_path, prev_path));
%                 waitforbuttonpress
%                 close all;
                if Q_prev < Q_new
                     % don't update values
                     no_improvement_counter = no_improvement_counter + 1;
                else
                     prev_path = cur_path;
                     cur_path = new_path;
                     Q_prev = Q_new; % store costs
                     if Q_diff < stop_condition
                        no_improvement_counter = no_improvement_counter + 1;
                     else
                        no_improvement_counter = 0;
                     end
                end
            end
            
            
        end
        
        
        function [noisey_trajs, noises, range] = noiseyTrajs(obj, alpha_path, noise, n_trajs)
            noisey_trajs = zeros(size(alpha_path, 1), size(alpha_path, 2), n_trajs);
            noises = zeros(size(alpha_path, 1), n_trajs);
            for i = 1:n_trajs
                [noisey_trajs(:, :, i), noises(:, i), range]  = obj.noisyTraj(alpha_path, obj.R_inv, noise);
            end
            obj.noisey_trajs_xy = cat(3, obj.noisey_trajs_xy, noisey_trajs);
            
            if obj.debugSTOMP1
                obj.plotNoiseyTrajs(alpha_path, noisey_trajs);
            end
            if obj.debugSTOMP2
                cla(figure(2));
                figure(2); hold on;
                for i = 1:n_trajs
                    xy_path = obj.xy_path_func(noisey_trajs(:,:,i));
                    plot(xy_path(:,1), xy_path(:,2), 'rx')
                end
                pause(0.1)
            end
        end
        
        function traj_costs = evalTrajs(obj, trajs)
            vels = cat(1, zeros(1,5,5), diff(trajs))/obj.dt; % finite difference velocity at each point
            accs = cat(1, diff(vels), zeros(1,5,5))/obj.dt; % finite difference acceleration at each point
            traj_costs = zeros(size(trajs, 1), size(trajs, 3));
            for i_trajs = 1:size(trajs,3)
                for i_timestep = 1:size(trajs, 1)
                    pos_cost = obj.stateCost(trajs(i_timestep, :, i_trajs));
                    % % this is my version of the obstacle cost
                    % get endpoint velocity in operational space
                    v1 = obj.vel_func{1}(trajs(i_timestep, 1:3, i_trajs))*vels(i_timestep, 1:3, i_trajs)';
                    v2 = obj.vel_func{2}(trajs(i_timestep, [1,4,5], i_trajs))*vels(i_timestep, [1,4,5], i_trajs)';
                    % multiply by end point velocity
                    traj_costs(i_timestep, i_trajs) = pos_cost * [norm(v1(1:2)), norm(v2(1:2))]';
                    
                    % % implement a torque cost
                end
            end
        end
        
        function traj_prob = probTrajs(obj, costs)
            % reducing cost magnitude and clipping
            % so that it doesn't saturate exp
%             if mean(costs(:)) > 10
%                 costs = costs ./ mean(costs(:));
%             end
%             costs(costs > 10) = 10; 
            lambda = 1/10;
            traj_prob = obj.probTrajs_normsoftmax(costs, lambda);            
        end
                    
        function state_cost = stateCost(obj, q)
            state_cost = obj.eval_state(q);
        end
        
        function path_cost = pathCost(obj, path)
            path_cost = obj.eval_path(path);
        end
        
    end
    
    methods(Static)
        function A = computeA(steps)
            % theta is how many controllable dofs
            % steps is number of steps in trajectory
            N = steps;
            % inbuilt function to make diagonal matrix
            A = full(gallery('tridiag',N,1,-2,1));
            % add row with one at top and bottom
            A = [1, zeros(1,size(A,2)-1); A; zeros(1,size(A,2)-1), 1];
        end
        
        function M = computeM(R_inv, N)
            M = R_inv./max(R_inv)/N;
        end
        
        function [noisy_traj, noise, range] = noisyTraj(alpha_path, R_inv, noise_amp)
            % generates a single noisy trajectory
            % scale amount of range based on joint movement
            range = abs(max(alpha_path) - min(alpha_path));
            noise = mvnrnd(zeros(1,size(alpha_path,1)), R_inv)';
            noisy_traj = alpha_path + noise .* range*noise_amp;
        end
        
        function [] = plotNoiseyTrajs(alpha_path, noisey_trajs)
            for iq = 1:size(noisey_trajs, 2)
                figure()
                hold on;
                plot(alpha_path(:,iq), 'ro');
                title(sprintf('Joint %d', iq));
                for i = 1:size(noisey_trajs, 3)
                    plot(noisey_trajs(:,iq, i), 'xb');
                end
            end
            hold off
        end
        
        function [] = plotCovariance(R_inv)
            figure();
            hold on;
            for i = 1:size(R_inv,2)
                plot(R_inv(:,i), 'x')
            end
            hold off
        end
            
        function traj_prob = probTrajs_softmax(costs, lambda)
            traj_prob = zeros(size(costs));
            % calculating softmax based on costs
            % higher cost should result in lower probability
            for i_timestep = 1:size(costs, 1)
                for i_trajs = 1:size(costs,2)
                    traj_prob(i_timestep, i_trajs) = exp(-1/lambda * costs(i_timestep, i_trajs));
                end
                traj_prob(i_timestep, :) = traj_prob(i_timestep,:) / sum(traj_prob(i_timestep,:));
            end
        end
        
        function traj_prob = probTrajs_normsoftmax(costs, lambda)
            traj_prob = zeros(size(costs));
            h = 1/lambda;
            % calculating softmax based on costs
            % normalizing costs based on range of costs for that timestep
            % higher cost should result in lower probability
            for i_timestep = 1:size(costs, 1)
                min_cost = min(costs(i_timestep,:));
                max_cost = max(costs(i_timestep,:));
                for i_trajs = 1:size(costs,2)
                    cost_norm = (costs(i_timestep, i_trajs) - min_cost)/(max_cost - min_cost);
                    if isnan(cost_norm) % in case of divide by zero
                        cost_norm = 1;
                    end
                    traj_prob(i_timestep, i_trajs) = exp(-h * cost_norm);
                end
                traj_prob(i_timestep, :) = traj_prob(i_timestep,:) / sum(traj_prob(i_timestep,:));
            end
        end
        
        function delta_q = deltaQ(traj_probs, noise, M)
            delta_q_noise = sum(traj_probs .* noise, 2);
            delta_q = M * delta_q_noise;
        end
        
        function path_adj = adjustPath(alpha_path, delta, range)
            joint_delta = delta * range; % adjusting for specific range of each joint
            path_adj = alpha_path + joint_delta;
        end
        
    end
end