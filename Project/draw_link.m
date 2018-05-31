classdef draw_link
    properties
        link
        figure
        ax
    end

    methods
        function obj = draw_link(link, f)
            obj.link = link;
            obj.figure = f;
            obj.ax = obj.figure.CurrentAxes;
        end
        
        function obj = draw(obj)
            x_0 = obj.link.pose_world(1);
            y_0 = obj.link.pose_world(2);
            theta = obj.link.pose_world(3);
            l = obj.link.length;
            plot(x_0, y_0, 'ro')
            x_e = x_0 + cos(theta) * l;
            y_e = y_0 + sin(theta) * l;
            
        end
        
    end


end