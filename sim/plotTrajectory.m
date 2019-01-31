%%
%--------------------------------------------------------------------------
%							    plotTrajectory.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Harsh Bhakta                                            	        
% @file_name				 : 		plotTrajectory.m														  
% @ Date                     : 	    11/29/18                                                     
% @ Discription				 :     This function helps plot the trajectory
%                                   of the Planner passed in
%
% @ Usage					:      plotTrajectory(Planner)    																						  
% @Revision					:  	   12/07/18                                                                                      
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
        case 'ppp'
            load('results/resultPPP.mat');
        otherwise
            
    end
    
    hold on
    if color_index == 1
        plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
        rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
        rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,obstacle.safeDistanceX + obstacle.Length,obstacle.safeDistanceY + obstacle.Width],'EdgeColor','m'); %safety region
        hold on
    
        plot(goal(1),goal(2), 'rx', 'LineWidth',2);
        hold on
    end
    
    plot(ydata(1, :), ydata(2, :), color(color_index), 'LineWidth', 2);
    hold on;
    
    %legend('obstacle', 'target')
    title('Trajectory Comparision')
end

