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
    %% MPC parameters information
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

    const_X_max = Inf;
    const_X_min = 0;

    const_Y_max = 10;
    const_Y_min = -10;

    const_Vx_max = 100;
    const_Vx_min = 0;

    const_Vy_max = 100;
    const_Vy_min = -100;

    const_ax_max = 2;
    const_ax_min = -2;

    const_ay_max = 2;
    const_ay_min = -2;

    mpcobj.OutputVariables(1).Max = const_X_max;
    mpcobj.OutputVariables(1).Min = const_X_min;

    mpcobj.OutputVariables(2).Max = const_Y_max;
    mpcobj.OutputVariables(2).Min = const_Y_min;

    mpcobj.OutputVariables(3).Max = const_Vx_max;
    mpcobj.OutputVariables(3).Min = const_Vx_min;

    mpcobj.OutputVariables(4).Max = const_Vy_max;
    mpcobj.OutputVariables(4).Min = const_Vy_min;

    mpcobj.OutputVariables(5).Max = const_ax_max;
    mpcobj.OutputVariables(5).Min = const_ax_min;

    mpcobj.OutputVariables(6).Max = const_ay_max;
    mpcobj.OutputVariables(6).Min = const_ay_min;



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
    yMPCMOVE = [];
    uMPCMOVE = [];

    Tsim = 10;



    x = x0;
    u=u0;

    xmpc = mpcstate(mpcobj);


    options = mpcmoveopt;
    for ct = 1:round(Tsim/Ts)+1

         % Update and store plant output.
        [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x,u);
        y = Cd * x + Dd * u;


        yMPCMOVE = [yMPCMOVE y];
        % Update constraints.
        [Min_y, Max_y]= ouputConstraint(x,1,obstacle);

        options.OutputMin =  [const_X_min;Min_y;const_Vx_min;const_Vy_min;const_ax_min;const_ay_min]';
        options.OutputMax =  [const_X_max;Max_y;const_Vx_max;const_Vy_max;const_ax_max;const_ay_max]';


        %mpcobj.OutputVariables(2).Max = Max_y;
        %mpcobj.OutputVariables(2).Min = Min_y;
        % Compute control actions.
        u = mpcmove(mpcobj,xmpc,y,ref,[],options);
        % Update and store plant state.
        x = plant.A*x + plant.B*u;
        uMPCMOVE = [uMPCMOVE u];
    end
    
    data = 'result.mat';
    save(data);
end