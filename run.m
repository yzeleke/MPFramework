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
    
    Ts = 0.02; %this is the sampling time....maybe we can put this on run params?

    %% Chose simulation environment and initialize
    init(Environment,options);
    
    %% Chose vehicle model
    switch Vmodel
        case 'pointmass' %can you look up a way to use both 'pointmass' and 'Pointmass'
                pointmass(Ts);
                
        case 'Dubin' %can you look up a way to use both 'pointmass' and 'Pointmass'
                Dubin(Ts);
        otherwise
            disp('Vehicle model not found in database')
    end
    

   
   %% Set simulation time
    
    %% Chose vehicle model
    switch Planner
        case 'MPC'
               MpcPlanner(SimTime);
        otherwise
            disp('Vehicle model not found in database')
    end
    

    %% plot results
    plotResult();
    
    %comment this line if you choose to save the data for later use
    delete *.mat

end