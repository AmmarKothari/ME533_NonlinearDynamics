classdef gripper
    properties
        links
        base_link % which link to start from -- lets ignore even length snakes for now
        base_pose
        num_links
        zero_link
        end_points
        end_link_names
    end
    
    methods
        function obj = gripper(links, zero_link)
            obj.links = links;
            obj.num_links = length(links);
            obj.zero_link = zero_link;
            obj.base_pose = zeros(1,3);
            obj = obj.endLinks();
        end
        
        function poses = get_poses(obj)
            poses = zeros(obj.num_links, 3);
            for i = 1:obj.num_links
                poses(i,:) = obj.links(i).pose;
            end
        end
        
        function obj = calc_poses(obj, alphas)
            if nargin < 2
                alphas = [obj.links.alpha];
            else
                % in a snake chain, there is one less joint than link so
                % adjust it like this
                alphas = [alphas(1:obj.zero_link-1), 0, alphas(obj.zero_link:end)];
                obj = obj.set_joints(alphas);
            end
            %start at '0' link
            forward_range = obj.zero_link+1:1:obj.num_links;
            backward_range = obj.zero_link-1:-1:1;
            %calculate position of zero link (if it can have some
            %orientation)
            obj.links(obj.zero_link) = obj.links(obj.zero_link).linkPosForward(alphas(obj.zero_link+1));
            for i = forward_range
                % find index of parent link
                parent_index = strcmp({obj.links.name} , obj.links(i).parent);
                if sum(parent_index) < 1
                    exception = MException('MyFunc:notValidSize', 'There is no parent link');
                    throw(exception)
                elseif sum(parent_index) > 1
                    exception = MException('MyFunc:notValidSize', 'There are multiple parent links!');
                    throw(exception)
                end
                obj.links(i) = obj.links(i).setProximal(obj.links(parent_index).distal);
                obj.links(i) = obj.links(i).linkPosForward(alphas(i+1));
            end
            for i = backward_range
                obj.links(i) = obj.links(i).setDistal(obj.links(i+1).proximal);
                obj.links(i) = obj.links(i).linkPosBackward(alphas(i));
            end
            
        end
        
        function obj = set_joints(obj, alpha_)
            for i = 1:obj.num_links
                obj.links(i).alpha = alpha_(i);
            end
        end
        
        function obj = endLinks(obj)
            obj.end_link_names = {};
            for i = 1:obj.num_links
                name = obj.links(i).name;
                parent = obj.links(i).parent;
                % if parent in list, remove from list
                parent_index = strcmp(parent, obj.end_link_names);
                if sum(parent_index) == 1
                    obj.end_link_names(parent_index) = [];
                end
                % if name not in list, add to list
                if sum(strcmp(name, obj.end_link_names)) == 0
                    obj.end_link_names{length(obj.end_link_names)+1} = name;
                end
            end
        end
        
        function end_points = endPoints(obj)
            end_points = [];
            for i = 1:length(obj.end_link_names)
                end_index = strcmp({obj.links.name} , obj.end_link_names{i});
                end_points = [end_points; obj.links(end_index).distal];
            end
        end
        
        function alphas = get_alphas(obj)
            alphas = [obj.links.alpha];
        end
        
        function alphas = get_alphas_world(obj)
            % returns the orientation of each link
            alphas = obj.get_poses;
            alphas = alphas(:,3);
        end
        
        function dist = dist_to_goal(obj, goal, al)
            f_arm = obj.calc_poses(al);
            tips = f_arm.endPoints();
            dist = tips(:,1:2) - goal;
        end
        
        function cost = dist_cost(obj, goal, alpha)
            dist = obj.dist_to_goal(goal, alpha);
            cost = sum(0.5 * dist(:).^2); % quadratic
        end
        
        function goal_alphas = invKin(obj, goal)
            dist_func = @(al) obj.dist_cost(goal, al);
            x0 = obj.get_alphas();
            % add joint constraints!
            goal_alphas = fmincon(dist_func, x0);
        end
    
        function obj = draw(obj, ax)
            for i = 1:obj.num_links
                obj.links(i).drawLink(ax);
            end
            axis(ax, 'equal')
        end
        
        function obj = draw_clear(obj)
            for i = 1:obj.num_links
                obj.links(i) = obj.links(i).clearLink();
            end
        end
        
        function Js = jacobian_func(obj)
            % returns functions to get 2 jacobians (one for each side of
            % gripper)
            J = @(l0,theta0,l1,theta1,l2,theta2) ...
                    [-l0*sin(theta0)-l1*sin(theta1+theta0)-l2*sin(theta0+theta1+theta2), -l1*sin(theta1+theta0)-l2*sin(theta0+theta1+theta2), -l2*sin(theta0+theta1+theta2);
                    l0*cos(theta0)+l1*cos(theta1+theta0)+l2*cos(theta0+theta1+theta2), l1*cos(theta1+theta0)+l2*cos(theta0+theta1+theta2),  l2*cos(theta0+theta1+theta2);
                    1,1,1];
            J1 = @(a) J(obj.links(1).h0(1), a(1), obj.links(2).h0(1), a(2), obj.links(3).h0(1), a(3));
            J2 = @(a) J(obj.links(1).h0(1), a(1), obj.links(4).h0(1), a(2), obj.links(5).h0(1), a(3));
            Js = {J1; J2};
        end
        
        function J = jacobian(obj)
           poses = reshape([obj.links.pose],3,[])';
           a_locals = reshape([obj.links.a],3,[])';
           J = GeoOps2D.jacobian_spatial(poses, a_locals);
        end
        
    end
    
    
    
    
end
    
    
    