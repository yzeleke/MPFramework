function [min_y,max_y] = ComputeConstraints(x,obstacle,safety,upper_bound,lower_bound)
%% Compute custom constraints for the obstacle.

    x1=x(1);
    x2=x(2);
    if (obstacle.X-safety < x1 < obstacle.X+safety) && x2>= obstacle.Y
        min_y = obstacle.Y+safety;
        max_y = upper_bound ;
    elseif (obstacle.X-safety < x1 < obstacle.X+safety) && x2 < obstacle.Y
        min_y = lower_bound;
        max_y = obstacle.Y-safety ;
    else
        min_y = lower_bound;
        max_y = upper_bound ;
    end
end
