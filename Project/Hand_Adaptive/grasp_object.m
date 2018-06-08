classdef grasp_object
    properties
        type
        center
        dims
        boundary
    end
    methods
        function obj = grasp_object(type)
            isValid = strcmp(type, {'rectangle', 'circle'});
            if sum(isValid) == 1
                obj.type = type;
            else
                exception = MException('MyFunc:notValidSize', 'There are multiple parent links!');
                throw(exception)
            end
        end
        
        function obj = setDims(obj, dims)
            obj.dims = reshape(dims, [1,2]);
        end
        
        function obj = setLocation(obj, center)
            obj.center = reshape(center, [1,2]);
        end
        
        function obj = setBoundary(obj)
                obj.boundary = [ obj.center-obj.dims;
                                obj.center-obj.dims.*[-1,1];
                                obj.center+obj.dims;
                                obj.center-obj.dims.*[1,-1];
                                obj.center-obj.dims;];
        end 
        
        function obj = draw(obj, ax)
            if strcmp(obj.type, 'rectangle')
                hold on
                    plot(ax, obj.boundary(:,1), obj.boundary(:,2), 'o-')
                hold off
            end
            if strcmp(obj.type, 'circle')
                obj.r = obj.dims(1);
            end
        end
    end
end