function [output, source, goal] = gridize(obstacle,resolution_x, resolution_y, lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y, source_input, goal_input)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
source =  zeros(1,2);
source(1) = (source_input(1) - lower_bound_x)/resolution_x + 1;
source(2) = (source_input(2) - lower_bound_y)/resolution_y + 1;
goal = zeros(1,2);
goal(1) = (goal_input(1) - lower_bound_x)/resolution_x + 1;
goal(2) = (goal_input(2) - lower_bound_y)/resolution_y + 1;

% source = ceil((source - [lower_bound_x,lower_bound_y])/resolution_x) + 1; 
% goal = ceil((goal - [lower_bound_x,lower_bound_y])/resolution_x) + 1; 


upper_bound_i = ceil((upper_bound_x - lower_bound_x)/resolution_x) + 1;
upper_bound_j = ceil((upper_bound_y - lower_bound_y)/resolution_y) + 1;
output = ones(upper_bound_i, upper_bound_j);


for i = 1:upper_bound_i
    for j = 1:upper_bound_j
        x = (i - 1)*resolution_x + lower_bound_x;
        y = (j - 1)*resolution_y + lower_bound_y;
        if (abs(y - obstacle.Y) < (0.5*obstacle.Width+obstacle.safeDistanceY))& (abs(x - obstacle.X) < (0.5*obstacle.Length+obstacle.safeDistanceX))
            output(i,j) = 0;
        end
    end
end
end

