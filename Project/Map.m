classdef Map
    properties
        cost_map
        obj_list
        x_limits
        y_limits
        x_range
        y_range
        step_size
    end
    methods
        function obj = Map()
            obj.obj_list = {};
        end
        function obj = limits(obj)
            obj.x_limits = [3 6];
            obj.y_limits = [0 4];
            obj.step_size = 0.1;
            obj.x_range = obj.x_limits(1):obj.step_size:obj.x_limits(2);
            obj.y_range = obj.y_limits(1):obj.step_size:obj.y_limits(2);
        end
        function obj = costMap(obj)
            obj = obj.limits();
            obj.cost_map = zeros(length(obj.x_range), length(obj.y_range));
            % could probably speed this up by checking mutliple points at
            % the same time
            for o = 1:length(obj.obj_list)
                for i1 = 1:length(obj.x_range)
                    for i2 = 1:length(obj.y_range)
                        [in, on] = inpolygon(obj.x_range(i1), obj.y_range(i2), obj.obj_list{o}.boundary(:,1), obj.obj_list{o}.boundary(:,2));
                        if in == 0 % not in or on boundary
                            obj.cost_map(i1,i2) = obj.cost_map(i1,i2) + 0;
                        else % in or on boundary
                            obj.cost_map(i1,i2) = obj.cost_map(i1,i2) + 1;
                        end
                    end
                end
            end
        end
        
        function distmap = addDistMap(obj, POIs)
%           add distance cost to existing cost map
            distmap = zeros(size(obj.cost_map));
            % for each point in map
            for i1 = 1:length(obj.x_range)
                for i2 = 1:length(obj.y_range)
                    % check distance to POIs
                    dist_vec = [obj.x_range(i1), obj.y_range(i2)] - POIs;
                    dist = vecnorm(dist_vec, 2, 2); % norm of each row
                    [~,min_idx] = min(dist); % only count distance to closest POI (is this the right thing to do?)
                    distmap(i1,i2) = dist(min_idx);
                end
            end
        end
        
        function noise_map = noiseCostMap(obj, noisey_objects, noise_std)
            % spread out cost as if object could inhabit multiple positions
            M = Map();
            for io = 1:length(obj.obj_list)
                noise_obj = cell(1, noisey_objects);
                for i = 1:noisey_objects
                    dims = obj.obj_list{io}.dims;
                    center = obj.obj_list{io}.center;
                    noise_obj{i} = obj.obj_list{io};
                    noise_obj{i} = noise_obj{i}.setDims(dims + randn(size(dims))*noise_std);
                    noise_obj{i} = noise_obj{i}.setLocation(center + randn(size(center))*noise_std);
                    noise_obj{i} = noise_obj{i}.setBoundary();
                    M = M.addObject(noise_obj{i});
                end
            end
            M = M.costMap();
            noise_map = M.cost_map;
%             figure();
%             ax = gca;
%             for i = 1:length(noise_obj)
%                 noise_obj{i}.draw(ax);
%             end    
        end
        
        function blur_map = blurCostMap(obj, filter_size, cost_map)
            % this can blur the edge of the object to deal with variation
            % in position
            % change b to change the amount of blurring
            if rem(filter_size, 2) == 0
                exception = MException('MyFunc:notValidSize', 'Filter size must be odd!');
                throw(exception)
            end
            os = floor(filter_size/2);
            blur = ones(filter_size,filter_size);
            blur = blur/length(blur(:));
            blur_map = conv2(blur, cost_map);
            blur_map = blur_map(1+os:end-os, 1+os:end-os);
%             blur_map = max(blur_map, cost_map); %ensures that a spot filled with object is 1
        end
        
        function obj = addObject(obj, Object)
            i = length(obj.obj_list);
            obj.obj_list{i+1} = Object;
        end
        
        function norm_map = normalizeCostMap(obj, costmap)
            % normalize map so values between 0 and 1
            norm_map = (costmap-min(costmap(:))) / (max(costmap(:)) - min(costmap(:)));
        end
    end
end