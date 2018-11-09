%%
%--------------------------------------------------------------------------
%										pointmass_config.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke                                            	        
% @file_name				 : 		 pointmass_config.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :      Vehicle model and configuratin file
%                                   point mass model
%
% @ Usage					:pointmass_config(0.02) for 0.02s sampling time    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function pointmass_config(Ts)
    %% inital conditions
    x0 = [0; 9; 10; 0;0;0]; 

    %set target state
    goal = [80; 0; 0; 0;0;0]; 

    u0= [2;-2];

    ref = [0,7,0,0,0,0];


    %% Road and Obstacle Information
    % In this example, assume that:
    %
    % * The road is straight and has |3| lanes.
    % * Each lane is |4| meters wide.
    % * The ego car drives in the middle of the center lane when not passing.
    % * Without losing generality, the ego car passes an obstacle only from the
    % left (fast) lane.
    %


    

    %%

    %initial input 
    %input takes speed and steering in that order
    u0 = [2; 2];
    
    %Ts = 0.02;
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
    plant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    plant.InputName = {'u_x','u_d'};
    plant.StateName = {'X','Y','V_x','V_y','a_x','a_y'};
    plant.OutputName = plant.StateName;
    
    filename = 'model.mat';
    save(filename)

end