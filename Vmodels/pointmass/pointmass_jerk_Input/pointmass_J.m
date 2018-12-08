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

function pointmass_J(Ts)

    %load environment
   load('environment.mat');
   
    % inital conditions
    model = 'pointmass_j';
    x0 = [5; 7; 30; 5; 0; 0]; 

    %set target state
    goal = [90; 7; 0; 0; 0; 0]; 

    %initial input 
    %input takes jerk in x and jerk in y
    u0 = [0.1; 0.1];

    ref = goal;


    %% Constraint information
     % state constraints
    upper_bound_x = Inf;
    lower_bound_x = 0;
    
    upper_bound_y = laneWidth*lanes;
    lower_bound_y = 0;
    
    upper_bound_speed_x =40;
    lower_bound_speed_x = 0;
    
    upper_bound_speed_y = 40;
    lower_bound_speed_y= -40;
    
    upper_bound_acc_x = 1;
    lower_bound_acc_x =-1;
    
    upper_bound_acc_y = 1;
    lower_bound_acc_y =-1;
    
    lower_bound_Jerk = -1.5;
    upper_bound_Jerk = 1.5;


    

    %%

   
    
    %Ts = 0.02;
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT_pointMass_J(Ts,x0,u0);
    plant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    plant.InputName = {'u_x','u_d'};
    plant.StateName = {'X','Y','V_x','V_y','a_x','a_y'};
    plant.OutputName = plant.StateName;
    
    filename = 'model.mat';
    save(filename)

end