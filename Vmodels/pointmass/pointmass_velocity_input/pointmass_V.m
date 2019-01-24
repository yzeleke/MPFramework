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

function pointmass_V(Ts)

    %load environment
   load('environment.mat');
   
    % inital conditions
    model = 'pointmass_v';
    x0 = [10; 10]; 

    %set target state
    goal = [90; 10]; 

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
    
    upper_bound_speed_y = 5;
    lower_bound_speed_y= -5;
    
    



    

    %%

   
    
    %Ts = 0.02;
    
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT_pointMass_V(Ts,x0,u0);
    plant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    plant.InputName = {'v_x','v_y'};
    plant.StateName = {'X','Y'};
    plant.OutputName = plant.StateName;
    
    filename = 'model.mat';
    save(filename)

end
