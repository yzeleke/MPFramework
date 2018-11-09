%%
%--------------------------------------------------------------------------
%										run.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke                                            	        
% @file_name				 : 		 run.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :      Motion Planning Framework by which a
%                                   user choose a planning algorithm and
%                                   a vehicle dynamics and run simulations
%
% @ Usage					: run('model','planner','environment',sim_time)    																						  
%@Revision					:  	11/5/18                                                                                      
%***************************************************************************


function run(Vmodel,Planner,Environment,options,SimTime)    

    %close all windows but do not clear workspace otherwise we loose all params.
    close all;

    %maybe we can have function like init() to load database;
    init(Environment,options);
    
    %% Chose vehicle model
    switch Vmodel
        case 'pointmass' %can you look up a way to use both 'pointmass' and 'Pointmass'
                Ts = 0.02; %this is the sampling time....maybe we can put this on run params?
                pointmass_config(Ts);
                
        otherwise
            disp('Vehicle model not found in database')
    end
    
   %% Chose simulation environment
   
   %% Set simulation time
    
    %% Chose vehicle model
    switch Planner
        case 'MPC'
               MpcPlanner();
        otherwise
            disp('Vehicle model not found in database')
    end
    

    
    %plot results....we might need a better plotting function setup
    plotResult();
    
    %comment this line if you choose to save the data for later use
    delete *.mat

end