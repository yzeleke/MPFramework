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

function Dubin(Ts)

    %load environment
   load('environment.mat');
    

    %% inital conditions
    model = 'Dubin';
    
    x0 = [5; 7; 0; 0]; 
    u0 = [70; 0];
    
    %set target state
    goal = [80; 7; 0; 0]; 

    ref = [80,7,0,0];


    %% Constraint information
     % state constraints
    upper_bound_x = Inf;
    lower_bound_x = 0;
    
    upper_bound_y = laneWidth*lanes;
    lower_bound_y = 0;
    
    upper_bound_theta =pi/6;
    lower_bound_theta = -pi/6;
    
    upper_bound_phi = pi/3;
    lower_bound_phi= -pi/3;
    
    upper_bound_speed = 70;
    lower_bound_speed= 0;
    

    
    %% 
    % Obtain a linear plant model at the nominal operating point and convert it
    % into a discrete-time model to be used by the model predictive controller.


    %initial input 
    %input takes speed and steering in that order
    u0 = [2; 2];
    
    %Ts = 0.02;
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT_Dubin(Ts,x0,u0);
    plant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    plant.InputName = {'Speed','Steering'};
    plant.StateName = {'X','Y','Theta','Phi'};
    plant.OutputName = plant.StateName;
    
    filename = 'model.mat';
    save(filename)

end