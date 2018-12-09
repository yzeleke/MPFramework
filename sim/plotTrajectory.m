%%
%--------------------------------------------------------------------------
%										run.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke and Harsh Bhakta                                            	        
% @file_name				 : 		 run.m														  
% @ Date                     : 	   12/05/18                                                     
% @ Discription				 :     This function helps plot the trajectory
%                                   on the 
%
% @ Usage					:    plotTrajectory(Planner)    																						  
% @Revision					:  	12/07/18                                                                                      
%***************************************************************************


function plotTrajectory(Planner, color_index)
    load('environment.mat');
    color = ["r", "g", "b", "y"];
    
    disp(color(color_index))
    disp(Planner)
    
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
    if color_index == 1
        plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
        rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
        hold on
    
        plot(goal(1),goal(2), 'rx', 'LineWidth',2);
        hold on
    end
    
    
    plot(ydata(1, :), ydata(2, :), color(color_index), 'LineWidth',2);
    hold on;
    
    legend('obstacle', 'target')
    title('Trajectory Comparision')
end

