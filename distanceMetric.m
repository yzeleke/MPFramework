%%
%--------------------------------------------------------------------------
%							    distanceMetric.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Harsh Bhakta                                            	        
% @file_name				 : 		distanceMetric.m														  
% @ Date                     : 	    12/08/18                                                     
% @ Discription				 :     This function helps plot the distance
%                                   difference of the Planner passed in
%
% @ Usage					:      plotTrajectory(Planner)    																						  
% @Revision					:  	   12/07/18                                                                                      
%***************************************************************************


function distanceMetric(Planner, color_index)
    load('environment.mat');
    color = ["r", "g", "b", "y"];
    
    switch lower(Planner)
        case 'mpc'
            load('results/resultMPC.mat');
        case 'rrt'
            load('results/resultRRT.mat');
        case 'astar'
            load('results/resultAstar.mat');
        otherwise
            
    end
    
    hold on
  
    %ydata(1, :) -> x   ydata(2, :) -> y
    %obstacle.X,obstacle.Y
    %Calculate the distance....
    
    %Allocate space for distance array
    %dist=cell(2, (numel(ydata)/2));
    
    %Calculate the distance
    for i=1:1:(numel(ydata)/2)
        x = ydata(1,i);
        y = ydata(2,i);
        
        p_x = (obstacle.X - x)^2;
        p_y = (obstacle.Y - y)^2;
        
        dist(i) = sqrt(p_x + p_y);
    end
    
    %Plot the result
    plot(dist, color(color_index), 'LineWidth',2);
    hold on;
   
    title('Distance to Obstacle Comparision')
end

