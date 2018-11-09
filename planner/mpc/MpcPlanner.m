%%
%--------------------------------------------------------------------------
%										MpcPlanner.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke                                            	        
% @file_name				 : 		 MpcPlanner.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :    Mpc planner model has to be defined and
%                                   saved as .mat file vefore calling 
%                                   this function
%
% @ Usage					:   MpcPlanner(model) model is the
%                                   name of .mat file    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function MpcPlanner()
    % Load environment
    load('environment.mat');
    %load vehilce model
    load('model.mat');
    % MPC Design at the Nominal Operating Point
    % Design a model predictive controller that can make the ego car maintain
    % a desired velocity and stay in the middle of the center lane.
    status = mpcverbosity('off');
    mpcobj = mpc(plant);

    % 
    % The prediction horizon is |25| steps, which is equivalent to 0.5 seconds.
    mpcobj.PredictionHorizon = 50;
    mpcobj.ControlHorizon = 15;

    %% Hard Constraints on System Dynamics 
    % To prevent the car from experiencing unrealstic speed, set constraints on
    % speed

 

    mpcobj.OutputVariables(1).Max = upper_bound_x;
    mpcobj.OutputVariables(1).Min = lower_bound_x;

    mpcobj.OutputVariables(2).Max = upper_bound_y;
    mpcobj.OutputVariables(2).Min = lower_bound_y;

    mpcobj.OutputVariables(3).Max = upper_bound_speed_x;
    mpcobj.OutputVariables(3).Min = lower_bound_speed_x;

    mpcobj.OutputVariables(4).Max = upper_bound_speed_y;
    mpcobj.OutputVariables(4).Min = lower_bound_speed_y;

    mpcobj.OutputVariables(5).Max = upper_bound_acc_x;
    mpcobj.OutputVariables(5).Min = lower_bound_acc_x;

    mpcobj.OutputVariables(6).Max = upper_bound_acc_y;
    mpcobj.OutputVariables(6).Min = lower_bound_acc_y;



    %% Adjust Weights
    % Since there are only two manipulated variables, to achieve zero
    % steady-state offset, you can choose only two outputs for perfect
    % tracking. In this example, choose the Y position and velocity by setting
    % the weights of the other two outputs (X and theta) to zero. Doing so lets
    % the values of these other outputs float.
    % 0.05 ? Low priority: Large tracking error acceptable
    % 0.2 ? Below-average priority
    % 1 ? Average priority ? the default. Use this value if nyc = 1.
    % 5 ? Above average priority
    % 20 ? High priority: Small tracking error desired

    mpcobj.Weights.OutputVariables = [0 5 0 0 0 0];
    mpcobj.Weights.ManipulatedVariables = [0 0];

    %%
    ydata = [];
    udata = [];

    Tsim = 10;



    x = x0;
    u=u0;

    xmpc = mpcstate(mpcobj);


    options = mpcmoveopt;
    for ct = 1:round(Tsim/Ts)+1

         % Update and store plant output.
        [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x,u);
        y = Cd * x + Dd * u;


        ydata = [ydata y];
        % Update constraints.
        [Min_y, Max_y]= ouputConstraint(x,1,obstacle);
        
       

        options.OutputMin =  [lower_bound_x;Min_y;lower_bound_speed_x;lower_bound_speed_y;lower_bound_acc_x;lower_bound_acc_y]';
        options.OutputMax =  [upper_bound_x;Max_y;upper_bound_speed_x;upper_bound_speed_y;upper_bound_acc_x;upper_bound_acc_y]';


        %mpcobj.OutputVariables(2).Max = Max_y;
        %mpcobj.OutputVariables(2).Min = Min_y;
        % Compute control actions.
        u = mpcmove(mpcobj,xmpc,y,ref,[],options);
        % Update and store plant state.
        x = plant.A*x + plant.B*u;
        
        %update obstacle location if obstacle is moving
        % obstacle = update_Obstacle(obstacle);
        udata = [udata u];
    end
    
    data = 'result.mat';
    save(data);
end