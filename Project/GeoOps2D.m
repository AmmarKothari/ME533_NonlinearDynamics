classdef GeoOps2D
    properties
        
    end
    methods (Static)
        function out = group(g)
            p = pose_class(g);
            out = p.group;
        end
        
        function out = pose(g)
            p = pose_class(g);
            out = p.p;
        end
        
        function out = LeftAction(g,h)
            h = pose_class(h); g = pose_class(g);
            out = h.group * g.group;
        end
        
        function out = RightAction(g,h)
            h = pose_class(h); g = pose_class(g);
            out = pose_class(g.group * h.group);
        end
        
        function p = inverseGroup(g)
            p = pose_class(g);
            [x,y,theta,~,~] = p.vals();
            group_T = [1, 0, -x;
                    0, 1, -y;
                    0, 0, 1];
            group_R = [cos(-theta), -sin(-theta), 0;
                    sin(-theta), cos(-theta), 0;
                    0, 0, 1];
            p = pose_class(group_T * group_R);
        end

        function ad = adjoint(g)
            p = pose_class(g);
            [x,y,~,c,s] = p.vals();
            ad = [c,-s,y;
                  s,c,-x;
                  0,0,1];
        end
        
        function ad_inv = adjoint_inv(g)
            p = pose_class(g);
            [x,y,~,c,s] = p.vals();
            ad_inv = [c, s, x*s-y*c;
                     -x, c, x*c+y*s;
                      0, 0, 1];           
            
        end
        
        function rl = right_lifted(g)
            p = pose_class(g);
            [x,y,~,~,~] = p.vals();
            rl = [1, 0, -y;
                0, 1, x;
                0, 0, 1];
        end
        
        function rl_inv = right_lifted_inv(g)
            p = pose_class(g);
            [x,y,~,~,~] = p.vals();
            rl_inv = [1, 0, y;
                    0, 1, -x;
                    0, 0, 1];
        end
        
        function J_spatial = jacobian_spatial(poses, a_locals)
            J_spatial = [];
            for i = 1:length(poses)
                J_spatial = [J_spatial; GeoOps2D.adjoint(poses(i,:)) * a_locals(i,:)'];
            end
        end
        
        function J_world = jacobian_world(pose, poses, a_locals)
            J_spatial = GeoOps2D.jacobian_spatial(poses, a_locals);
            TeRg = GeoOps2d.right_lifted(pose);
            J_world = TeRg * J_spatial;
        end
        
    end
end
