classdef mapCell
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dx %for dijkstra
        dy %for dijkstra
        x
        y
        theta
        closed
        cost2d
        cost3d
    end
    
    methods
        function obj = mapCell(incost2d,incost3d, obst)
            obj.closed = obst;
            obj.cost2d = incost2d;
            obj.cost3d = incost3d;
        end
        
        function outputArg = getKey(obj, cellX, cellY, targetX, targetY)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.cost2d+abs(targetX-cellX)+abs(targetY-cellY);
        end
    end
end

