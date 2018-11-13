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

function init_parkinglot(options)
    
    %% Parking Lot Information
    parking_rows = 2;
    parking_space_per_row = 4;


%     %% 
%     % The obstacle in this example is a nonmoving object in the middle of the
%     % center lane with the same size as the ego car.
%     obstacle = struct;
%     obstacle.Length = 5;
%     obstacle.Width = 2;
% 
%     %% 
%     % Place the obstacle |50| meters down the road.
%     obstacle.X = 60;
%     obstacle.Y = 7;
%     
%     
%    
% 
%     %%
%     % Create a virtual safe zone around the obstacle so that the ego car does
%     % not get too close to the obstacle when passing it. The safe zone is
%     % centered on the obstacle and has a:
%     %
%     % * Length equal to two car lengths.
%     % * Width equal to two lane widths.
%     %
%     obstacle.safeDistanceX = obstacle.Length;
%     obstacle.safeDistanceY = obstacle.Width;
%     obstacle = obstacleGenerateObstacleGeometryInfo(obstacle);

    if ~isempty(options)
        %change the stuff here 
    end
    
    filename = 'environment.mat';
    save(filename)
    
    
end

