function [min_y, max_y] = ouputConstraint(x,detection,obstacle)
    %% Compute custom constraints for the obstacle.

    % Load environment
    load('environment.mat');
    %#codegen
    egoX = x(1);
    egoY = x(2);



    % Compute constraints only if an obstacle is detected. Otherwsie, set
    % constraint to lower road boundary (the inactive constraint).
    if detection
        slope = ( (obstacle.rlSafeY - egoY)/(obstacle.rlSafeX - egoX) );
        % If ego car is to the left of the obstacle
        if (egoX<=obstacle.rlSafeX)
            % if the ego car is already in the adjacent lane, use the safety
            % zone as the constraint.
            if (egoY>obstacle.rlSafeY)
                min_y = obstacle.rlSafeY+2;
                max_y = upper_bound_y;
                
                %if ego car is below the obstacle but to the left
%             elseif (egoY<obstacle.rrSafeY)
%                 min_y = lower_bound_y+2;
%                 max_y = obstacle.rrSafeY;
            else
                % The ego car must be above the line formed from the ego car to
                % safe zone corner for left passing.

                max_y = upper_bound_y;
                min_y= slope*(egoX)-slope*obstacle.rlSafeX+obstacle.rlSafeY + 2; %just add extra space of 2 'safety'
            end
        % If the ego car is parallel to the obstacle, and passing to the left
        % side
        elseif  (egoX>obstacle.rlSafeX) && (egoX<=obstacle.flSafeX) &&(egoY>obstacle.rlSafeY)
            min_y = obstacle.rlSafeY+4;
            max_y = upper_bound_y;
        % If the ego car is parallel to the obstacle, and passing to the right
        % side of the obstacle
        elseif  (egoX>obstacle.rlSafeX) && (egoX<=obstacle.flSafeX) && (egoY<obstacle.rlSafeY)
             min_y = lower_bound_y+2;
             max_y = obstacle.rrSafeY;

        else 
             min_y = lower_bound_y+2;
             max_y = upper_bound_y;
        end
    else
        min_y = lower_bound_y;
        max_y = upper_bound_y;
    end
end

