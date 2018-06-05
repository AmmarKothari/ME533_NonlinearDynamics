classdef pose_class
    properties
        group
        p
    end
    methods
        function obj = pose_class(g)
            if isa(g, 'pose_class')
                obj.group = g.group;
                obj.p = g.p;
            else
                obj.group = obj.group_func(g);
                obj.p = obj.poseFromMatrix(g);
            end
        end
        
        function [x,y,theta,c,s]=vals(obj)
            x = obj.p(1);
            y = obj.p(2);
            theta = obj.p(3);
            c = cos(theta);
            s = sin(theta);
        end
    end
    
    methods (Static)
        function out = group_func(g)
            if all(size(g) == [1,3])
                x = g(1);
                y = g(2);
                theta = g(3);
                out = [cos(theta), -sin(theta), x;
                        sin(theta), cos(theta), y;
                        0, 0, 1];
            elseif all(size(g) == [3,3])
                out = g;
            else
                exception = MException('MyFunc:notValidSize', 'Array is not a valid size');
                throw(exception)
            end
        end
        
        function g = poseFromMatrix(G)
            if all(size(G) == [3,3])
                g = zeros(1,3);
                g(1) = G(1,3);
                g(2) = G(2,3);
                g(3) = atan2(G(2,1), G(1,1));
            elseif all(size(G) == [1,3])
                g = G;
            else
                exception = MException('MyFunc:notValidSize', 'Array is not a valid size');
                throw(exception)
            end
        end
    end
end
    
    
    