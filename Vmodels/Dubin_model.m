function Dubin_model()
%% Vehicle Model
% The ego car has a rectagular shape with a length of 5 meters and width of
% 2 meters. The model has four states:
%
% * $x$   -  Global X position of the car center
% * $y$   -  Global Y position of the car center
% * $v_x$ -  longitude velocity (positive when going right, negative when left)
% * $v_y$ -  latitude velocity (positive when going up, negative when down)
% * $a_x$ -  longitude accelaration
% * $a_y$ -  latitude accelaration
%
% There are two manipulated variables:
%
% * $u_x$ - derivative of a_x 
% * $u_y$ - derivative of a_y
%
% Use a simple linear model to describe the dynamics of the ego car:
% 
% $$\begin{array}{l}
% \dot x    =  v_x
% \dot y    =  v_y
% \dot v_x  =  a_x  
% \dot v_y  =  a_y
% \dot a_x  =  u_x
% \dot a_y  =  u_y

%
% Also, assume all the states are measurable. At the nominal operating
% point

%clc;close all;


%set initial condition
x0 = [5; 0; 10; 0;0;0]; 

%set target state
goal = [80; 0; 0; 0;0;0]; 

%initial input 
%input takes speed and steering in that order
u0 = [2; 2];

%% 
% Obtain a linear plant model at the nominal operating point and convert it
% into a discrete-time model to be used by the model predictive controller.
Ts = 0.02;
[Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x0,u0);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
dsys.InputName = {'u_x','u_d'};
dsys.StateName = {'X','Y','V_x','V_y','a_x','a_y'};
dsys.OutputName = dsys.StateName;

%% Road and Obstacle Information
% In this example, assume that:
%
% * The road is straight and has |3| lanes.
% * Each lane is |4| meters wide.
% * The ego car drives in the middle of the center lane when not passing.
% * Without losing generality, the ego car passes an obstacle only from the
% left (fast) lane.
%
lanes = 3;
laneWidth = 4;

%% 
% The obstacle in this example is a nonmoving object in the middle of the
% center lane with the same size as the ego car.
obstacle = struct;
obstacle.Length = 5;
obstacle.Width = 2;

%% 
% Place the obstacle |50| meters down the road.
obstacle.X = 40;
obstacle.Y = 0;

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

%%
% In this example, assume that the lidar device can detect an obstacle |30|
% meters in front of the vehicle.
obstacle.DetectionDistance = 10000;

%%
% Plot the following at the nominal condition:
%
% * Ego car - Green dot with black boundary
% * Horizonal lanes - Dashed blue lines
% * Obstacle - Red |x| with black boundary
% * Safe zon - Dashed red boundary.
%

f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,1);

%% MPC Design at the Nominal Operating Point
% Design a model predictive controller that can make the ego car maintain
% a desired velocity and stay in the middle of the center lane.
status = mpcverbosity('off');
mpcobj = mpc(dsys);

%% 
% The prediction horizon is |25| steps, which is equivalent to 0.5 seconds.
mpcobj.PredictionHorizon = 50;
mpcobj.ControlHorizon = 15;

%% Hard Constraints on System Dynamics 
% To prevent the car from experiencing unrealstic speed, set constraints on
% speed

mpcobj.OutputVariables(3).Max = 100;
mpcobj.OutputVariables(3).Min = 0;

mpcobj.OutputVariables(4).Max = 100;
mpcobj.OutputVariables(4).Min = -100;
%%
% Similarly, add a hard constraint of 6 degrees per sec on the steering
% angle rate of change.
%mpcobj.ManipulatedVariables(2).RateMin = ;
%mpcobj.ManipulatedVariables(2).RateMax = ;


%%
% Scale the throttle and steering angle by their respective operating
% ranges.
%mpcobj.ManipulatedVariables(1).ScaleFactor = 1;
%mpcobj.ManipulatedVariables(2).ScaleFactor = 0.2;




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

mpcobj.Weights.OutputVariables = [0 0 0 0 0 0];
mpcobj.Weights.ManipulatedVariables = [0 0];

%% 
% Update the controller with the nominal operating condition. For a
% discrete-time plant:
%
% * |U = u0|
% * |X = x0|
% * |Y = Cd*x0 + Dd*u0|
% * |DX = Ad*X0 + Bd*u0 - x0|
%
mpcobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);

%% Specify Mixed I/O Constraints for Obstacle Avoidance Maneuver
% There are different strategies to make the ego car avoid an obstacle on
% the road. For example, a real-time path planner can compute a new path
% after an obstacle is detected and the controller follows this path.
%
% In this example, use a different approach that takes advantage of the
% ability of MPC to handle contraints explicitly. When an obstacle is
% detected, it defines an area on the road (in terms of constraints) that
% the ego car must not enter during the prediction horizon. At the next
% control interval, the area is redefined based on the new positions of
% the ego car and obstacle until passing is completed.
%
% To define the area to avoid, use the following mixed input/output
% constraints:
%
%   E*u + F*y <= G
% 
% where |u| is the manipulated variable vector and |y| is the output
% variable vector. You can update the constraint matrices |E|, |F|, and |G|
% when the controller is running.

%%
% The first constraint is an upper bound on $y$ ($y \le 6$ on this
% three-lane road).
E1 = [0 0];
F1 = [0 1 0 0 0 0]; 
G1 = 10;

%%
% The second constraint is a lower bound on $y$ ($y \ge -6$ on this
% three-lane road).
E2 = [0 0];
F2 = [0 -1 0 0 0 0]; 
G2 = 10;

%%
% The third constraint is for obstacle avoidance. Even though no obstacle
% is detected at the nominal operating condition, you must add a "fake"
% constraint here because you cannot change the dimensions of the
% constraint matrices at run time. For the fake constraint, use a
% constraint with the same form as the second constraint.
E3 = [0 0];
F3 = [0 -1 0 0 0 0]; 
G3 = 10;

%%
% Specify the mixed input/output constraints in the controller using the
% |setconstraint| function.
setconstraint(mpcobj,[E1;E2;E3],[F1;F2;F3],[G1;G2;G3]);

%% Simulate Controller
% In this example, you use an adaptive MPC controller because it handles
% the nonlinear vehicle dynamics more effectively than a traditional MPC
% controller. A traditional MPC controller uses a constant plant model.
% However, adaptive MPC allows you to provide a new plant model at each
% control interval. Because the new model describes the plant dynamics more
% accurately at the new operating condition, an adaptive MPC controller
% performs better than a traditional MPC controller.
%
% Also, to enable the controller to avoid the safe zone surrounding the
% obstacle, you update the third mixed constraint at each control interval.
% Basically, the ego car must be above the line formed from the ego car to
% the upper left corner of the safe zone. For more details, open
% |obstacleComputeCustomConstraint|.

%%
% Use a constant reference signal.
refSignal = goal;


%% Custom cost function
mpcobj.Optimizer.CustomCostFcn = true;
%%
% Initialize plant and controller states.
x = x0;
u = u0;
egoStates = mpcstate(mpcobj);


%%
%code generation tools
[configData,stateData,onlineData] = getCodeGenerationData(mpcobj);

%%
% The simulation time is |4| seconds.
T = 0:Ts:0.5;

%%
% Log simulation data for plotting.
saveSlope = zeros(length(T),1);
saveIntercept = zeros(length(T),1);
ympc = zeros(length(T),size(Cd,1));
umpc = zeros(length(T),size(Bd,2));
Jerk = zeros(length(T),2);
%%
% Run the simulation.
for k = 1:length(T)
    % Obtain new plant model and output measurements for interval |k|.
    [Ad,Bd,Cd,Dd,U,Y,X,DX] = obstacleVehicleModelDT(Ts,x,u);
    measurements = Cd * x + Dd * u;
    
    %code gen tool
    onlineData.signals.ym = measurements ;

    ympc(k,:) = measurements';
    
    Jerk(k,1) = u(1);
    Jerk(k,2) = u(2);
    
    % Determine whether the vehicle sees the obstacle, and update the mixed
    % I/O constraints when obstacle is detected.
    detection = obstacleDetect(x,obstacle,laneWidth);
    [E,F,G,saveSlope(k),saveIntercept(k)] = ...
        obstacleComputeCustomConstraint(x,detection,obstacle,laneWidth,lanes); 
   
    % Prepare new plant model and nominal conditions for adaptive MPC.
    newPlant = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
    newNominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
    
    % Prepare new mixed I/O constraints.
    options = mpcmoveopt;
    options.CustomConstraint = struct('E',E,'F',F,'G',G);
    
    % Compute optimal moves using the updated plant, nominal conditions,
    % and constraints.
    %refSignal(1) = x(1); 
    [u,Info] = mpcmoveAdaptive(mpcobj,egoStates,newPlant,newNominal,...
        measurements,refSignal,[],options);
    
    % Compute control actions.
    [u_codegen,statedata] = mpcmoveCodeGeneration(configData,stateData,onlineData);
    
    umpc(k,:) = u';
    
    % Update the plant state for the next iteration |k+1|.
    x = Ad * x + Bd * u;
end

mpcverbosity(status);

%% Code Generation
end
