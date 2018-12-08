%%
%--------------------------------------------------------------------------
%										run.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke and Harsh Bhakta                                            	        
% @file_name				 : 		 run.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :      Motion Planning Framework by which a
%                                   user choose a planning algorithm and
%                                   a vehicle dynamics and run simulations
%
% @ Usage					: run('model','planner','environment', 'options', sim_time, flag)    																						  
%@Revision					:  	11/5/18                                                                                      
%***************************************************************************

%%
function run(Vmodel,Planner,Environment,options,SimTime, flag)    

    %close all windows but do not clear workspace otherwise we loose all params.
    close all;
    
    Ts = 0.02; %this is the sampling time....maybe we can put this on run params?

    %% Chose simulation environment and initialize
    init(Environment,options);
    
    %% Chose vehicle model
    switch lower(Vmodel)
        case 'pointmass_v' 
                pointmass_V(Ts);
        case 'pointmass_j' 
                pointmass_J(Ts);
        case 'dubin' 
                Dubin(Ts);
        otherwise
            disp('Vehicle model not found in database')
    end
    

   
   %% Set simulation time
    switch flag
        case 0
            switch lower(Planner)
                case 'mpc'
                    MpcPlanner(SimTime);
                    plotResult();
                case 'rrt'
                    RRTPlanner(SimTime);
                    plotResult();
                case 'astar'
                    AstarPlanner(SimTime);
                    plotResult();
                otherwise
                    disp('Planner not found in database')
            end
        otherwise
<<<<<<< HEAD
            %plotTrajectory(SimTime)
=======
            switch lower(Planner)
                case 'mpc'
                    MpcPlanner(SimTime);
                    plotTrajectory(Planner)
                case 'rrt'
                    RRTPlanner(SimTime);
                    plotTrajectory(Planner)
                case 'astar'
                    AstarPlanner(SimTime);
                    plotTrajectory(Planner)
                otherwise
                    disp('Planner not found in database')
            end
            
>>>>>>> origin/master
    end
    %% Chose vehicle model
    
    

    %% plot results
    %plotResult();
    
    %comment this line if you choose to save the data for later use
    delete *.mat

end
