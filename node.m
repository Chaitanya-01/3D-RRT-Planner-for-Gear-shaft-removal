classdef node
    properties
        q
        parent
%         children
    end
    methods
        function obj = node(q)
            obj.q = q;
            obj.parent = -1;
%             obj.children = [];
        end
    end
end


% 
% % Add parent node information
%     % Add costs for the node
%     properties
%         pose
%     end
%     methods
%         function obj = node(pose)
%             obj.pose = pose;
%         end
%         function distance = dist(obj,pose2)
%             distance = sqrt((obj.pose(1)-pose2(1))^2+(obj.pose(2)-pose2(2))^2+(obj.pose(3)-pose2(3))^2);
%         end
%     end