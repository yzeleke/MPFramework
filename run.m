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


function run(Vmodel,Planner,Environment,SimTime)

    %maybe we can have function like init() to load database;
    
    
    %% Chose vehicle model
    switch Vmodel
        case 'pointmass' %can you look up a way to use both 'pointmass' and 'Pointmass'
                Ts = 0.02; %this is the sampling time....maybe we can put this on run params?
                model = pointmass_config(Ts);
                load('pointmass_init');
                
        otherwise
            disp('Vehicle model not found in database')
    end
    
   %% Chose simulation environment
   
   %% Set simulation time
    
    %% Chose vehicle model
    switch Planner
        case 'MPC'
                data = MpcPlanner(model);
        otherwise
            disp('Vehicle model not found in database')
    end
    

    
    %plot results....we might need a better plotting function setup
    plotResult(data);

end