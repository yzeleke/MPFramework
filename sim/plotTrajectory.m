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


function plotTrajectory(Planner)
    load('environment.mat');
    
    switch lower(Planner)
        case 'mpc'
            load('results/resultMPC.mat');
        case 'rrt'
            load('results/resultRRT.mat');
        case 'astar'
            load('results/resultAstar.mat');
        otherwise
            
    end
    
    
    %goal = [90, 7];  %This is here because RRT has an error which changes the goal
    figure(1)
    
    plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
    rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
    hold on
    
    plot(goal(1),goal(2), 'rx', 'LineWidth',2);
    hold on
    
    plot(ydata(1, :), ydata(2, :), 'y', 'LineWidth',2);
    hold on;
    
    legend('obstacle', 'target')
    title('Trajectory Comparision')
end

