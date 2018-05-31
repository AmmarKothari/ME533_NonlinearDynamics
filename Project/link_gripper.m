classdef link_gripper
    properties
        h, h_poi, a, h0
        index
        zero_pose
        alpha, alpha_dot
        pose
        proximal, distal
        name, parent
        c, plot_handles
        
        % input desired velocity and output acceleration
        alpha_dot_desired
        kp, ki
        alpha_dot_dot
        e, e_prev, e_total
    end
    
    methods
        function obj = link_gripper(name, parent, a, h, c, h_poi)
            obj.name = name; % name of this link
            obj.parent = parent; % name of parent link
            obj.h0 = h;
            obj.h = GeoOps2D.group(h);
            obj.h_poi = GeoOps2D.group(h_poi);
            obj.a = a;
            obj.pose = GeoOps2D.group([0,0,0]);
            obj.zero_pose = GeoOps2D.group([0,0,0]);
            obj.alpha = 0;
            obj.c = c;
            obj.alpha_dot_desired = 0;
            obj.kp = 10;
            obj.ki = 1;
            obj.alpha_dot = 0;
            obj.alpha_dot_dot = 0;
            obj.e_total = 0;
            obj.plot_handles=[];
        end
        function obj = setZero(obj, zero_pose)
            obj.zero_pose = zero_pose;
        end
        function obj = setProximal(obj, zero_pose)
            obj.zero_pose = zero_pose;
        end
        function obj = setDistal(obj, zero_pose)
            obj.zero_pose = zero_pose;
        end
        function obj = setAlphaDotDesired(obj, alpha_dot_desired)
            obj.alpha_dot_desired = alpha_dot_desired;
        end
        function obj = calcAlphaDD(obj)
            % calculates alpha_dd for a given error in alpha_d
            e = obj.alpha_dot_desired - obj.alpha_dot;
            obj.e_total = obj.e_total + e;
            obj.alpha_dot_dot = obj.kp * e;
            if sign(e) == sign(obj.e_total)
                obj.alpha_dot_dot = obj.alpha_dot_dot + obj.ki * obj.e_total;
%             else
%                 obj.alpha_dot_dot = obj.alpha_dot_dot + obj.ki * sign(e)*abs(obj.e_total);
            end
        end
        function obj = clearError(obj)
            obj.e_total = 0;
        end
        function obj = step(obj, dt)
            % runs system forward one time step
            obj.alpha_dot = obj.alpha_dot + dt*obj.alpha_dot_dot;
            obj.alpha_ = obj.alpha_ + dt*obj.alpha_dot;
        end
        function obj = linkPosForward(obj, alpha_)
            obj.alpha = alpha_;
            % figure out how the base pose changes
            h_rot = [obj.h(1, 1:2), 0; obj.h(2,1:2), 0; 0 0 1];
            remainder_h = [1 0 obj.h(1,3); 0 1 obj.h(2,3); 0 0 1]; % no additional rotation from base
            frame_rot = GeoOps2D.RightAction(h_rot, obj.a * obj.alpha);
            obj.proximal = GeoOps2D.RightAction(obj.zero_pose, frame_rot).p;
            obj.pose = GeoOps2D.RightAction(obj.proximal, remainder_h).p;
            obj.distal = GeoOps2D.RightAction(obj.pose, remainder_h).p;
        end
        function obj = linkPosBackward(obj, alpha_)
            obj.alpha = alpha_;
            % figure out how the base pose changes
            h_rot = [obj.h(1, 1:2), 0; obj.h(2,1:2), 0; 0 0 1];
            remainder_h = [1 0 obj.h(1,3); 0 1 obj.h(2,3); 0 0 1]; % no additional rotation from base
            frame_rot = RightAction(h_rot, obj.a * obj.alpha);
            obj.distal = poseFromMatrix(RightAction(obj.zero_pose, frame_rot));
            obj.pose = poseFromMatrix(RightAction(obj.distal, remainder_h));
            obj.proximal = poseFromMatrix(RightAction(obj.pose, remainder_h));
        end
        
        function obj = drawLink(obj, ax)
            X = [obj.proximal(1), obj.distal(1)];
            Y = [obj.proximal(2), obj.distal(2)];
            hold on
            % plots line
            obj.plot_handles(1) = plot(ax, X, Y, obj.c, 'LineWidth', 8);
            % plots center
            obj.plot_handles(2) = plot(ax, obj.pose(1), obj.pose(2), '*k');
            % plot proximal
            obj.plot_handles(3) = plot(ax, obj.proximal(1), obj.proximal(2), '^k');
            % plots distal
            obj.plot_handles(4) = plot(ax, obj.distal(1), obj.distal(2), 'sk');
            hold off
        end
        
        function obj = clearLink(obj)
            for i = 1:length(obj.plot_handles)
                delete(obj.plot_handles(i))
            end
        end
        
        function obj = drawDistalPose(obj, ax)
            plotPose(ax, obj.distal, 1);
        end
        
        function obj = drawPose(obj, ax, h)
            % plots a cosys at some point on link
            plotPose(ax, poseFromMatrix(rightAction(obj.pose, h)), 1);
        end
        
        function obj = drawArrow(obj, ax, arrow_params)
            poi = poseFromMatrix(rightAction(obj.pose, obj.h_poi));
            hold on;
            quiver3(ax, poi(1), poi(2), poi(3), arrow_params(1), arrow_params(2), arrow_params(3), 'linewidth',5);
            hold off;
        end
    end
    
    
    
end