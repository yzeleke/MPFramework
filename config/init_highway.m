%%
%--------------------------------------------------------------------------
%										init.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke and Harsh Bhakta                                           	        
% @file_name				 : 		 init_highway.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :    Initializes environment for simulation
%
% @ Usage					:   init_highway(varargin), i,e, init('highway','len',3...)    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function init_highway(options)
    
    %% Road and Obstacle Information
    % In this example, assume that:
    %
    % * The road is straight and has |3| lanes.
    % * Each lane is |4| meters wide.
    % * The ego car drives in the middle of the center lane when not passing.
    % * Without losing generality, the ego car passes an obstacle only from the
    % left (fast) lane.
    %
    lanes = 6;
    laneWidth = 2;


    %% 
    % The obstacle in this example is a nonmoving object in the middle of the
    % center lane with the same size as the ego car.
    obstacle = struct;
    obstacle.Length = 5;
    obstacle.Width = 2;

    %% 
    % Place the obstacle |50| meters down the road.
    obstacle.X = 60;
    obstacle.Y = 7;
    
    
    
    
    %% state constraints
    upper_bound_x = Inf;
    lower_bound_x = 0;
    
    upper_bound_y = laneWidth*lanes;
    lower_bound_y = 0;
    
    upper_bound_speed_x =70;
    lower_bound_speed_x = 0;
    
    upper_bound_speed_y = 70;
    lower_bound_speed_y= -70;
    
    upper_bound_acc_x = 2;
    lower_bound_acc_x =-2;
    
    upper_bound_acc_y = 2;
    lower_bound_acc_y =-2;

    %%
    % Create a virtual safe zone around the obstacle so that the ego car does
    % not get too close to the obstacle when passing it. The safe zone is
    % centered on the obstacle and has a:
    %
    % * Length equal to two car lengths.
    % * Width equal to two lane widths.
    %
    obstacle.safeDistanceX = obstacle.Length;
    obstacle.safeDistanceY = obstacle.Width;
    obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);

    if ~isempty(options)
        %change the stuff here 
    end
    
    filename = 'environment.mat';
    save(filename)
    
    
end