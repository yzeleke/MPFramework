function [output, source, goal] = gridize(obstacle,resolution_x, resolution_y, lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y, source, goal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

source = ceil((source - [lower_bound_x,lower_bound_y])/resolution_x) + 1; 
goal = ceil((goal - [lower_bound_x,lower_bound_y])/resolution_x) + 1; 


upper_bound_i = ceil((upper_bound_x - lower_bound_x)/resolution_x) + 1;
upper_bound_j = ceil((upper_bound_y - lower_bound_y)/resolution_y) + 1;
output = ones(upper_bound_i, upper_bound_j);


for i = 1:upper_bound_i
    for j = 1:upper_bound_j
        x = (i - 1)*resolution_x;
        y = (j - 1)*resolution_y;
        if (abs(x - obstacle.X) < 0.5*obstacle.Length)& (abs(y - obstacle.Y) < 0.5*obstacle.Length)
            output(i,j) = 0;
        end
    end
end
end

