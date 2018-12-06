function [min_y, max_y] = ouputConstraintConservative(x,detection,obstacle)
    %% Compute custom constraints for the obstacle.

    % Load environment
    load('environment.mat');
    load('model.mat');
    
    %#codegen
    egoX = x(1);
    egoY = x(2);
    
    % slack is used as safety/wiggle room
    slack =0.2;



    % Compute constraints only if an obstacle is detected. Otherwsie, set
    % constraint to lower road boundary (the inactive constraint).
    if detection
        
        % If ego car is to the left of the obstacle pass to the left of
        % obstacle
        if (egoX<=obstacle.flSafeX)
            
                min_y = obstacle.flSafeY+slack;
                max_y = upper_bound_y;
        else 
                 min_y = lower_bound_y;
                 max_y = upper_bound_y;
        end
  
    end
end

