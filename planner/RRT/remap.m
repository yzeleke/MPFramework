function [output] = remap(path,resolution_x, resolution_y, lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
output1 = (path(:,1)-1).*resolution_x + lower_bound_x;
output2 = (path(:,2)-1).*resolution_y + lower_bound_y;
output = [output1, output2];
end

