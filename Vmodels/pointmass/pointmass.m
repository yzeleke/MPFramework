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

function pointmass(Ts)

    %load environment
   load('environment.mat');
   
    % inital conditions
    model = 'pointmass';
    x0 = [5; 7; 10; 0; 0; 0]; 

    %set target state
    goal = [80; 7; 0; 0; 0; 0]; 

    u0= [2;-2];

    ref = [80,7,0,0,0,0];


    %% Constraint information
     % state constraints
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

    %initial input 
    %input takes speed and steering in that order
    u0 = [2; 2];
    
    %Ts = 0.02;
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT_pointMass(Ts,x0,u0);
    plant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    plant.InputName = {'u_x','u_d'};
    plant.StateName = {'X','Y','V_x','V_y','a_x','a_y'};
    plant.OutputName = plant.StateName;
    
    filename = 'model.mat';
    save(filename)

end