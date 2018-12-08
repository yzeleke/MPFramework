%%
%--------------------------------------------------------------------------
%										run.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Harsh Bhakta                                            	        
% @file_name				 : 		MpFramework.m														  
% @ Date                     : 	    12/07/18                                                     
% @ Discription				 :      Motion Planning Framework by which a
%                                   user choose a planning algorithm and
%                                   a vehicle dynamics and run simulations
%
% @ Arguments                ~ toDo - What should the framework do?
%                                       -> "compare" - use this as input to compare planners
%                                       -> "simulation" - use this to simulate one of the planner
%
%                            ~ model - What model should be used to compare or simulate planners
%                                       -> "pointmass_v" - pointmass model that doesn't include jerk 
%                                       -> "pointmass_j" - pointmass model that includes jerk and velocity
%                                       -> "dubin" - dubins model
%
%                            ~ planner - What planner/s to compare or simulate?
%                                       -> "mpc" - MPC planner 
%                                       -> "rrt" - RRT planner
%                                       -> "astar" - Astar planner
%                                  NOTE:- This needs to be in an array form ["mpc", "rrt", "astar"] 
%
%                            ~ environment - What environment the planner and model should be ran in?
%                                       -> "H" - highway environment 
%                                       -> "P" - Parking Lot  (Coming soon)
%
%                            ~ options - Options to modify Environment?
%                                       -> pass in empty array for now... e.g []
%
%                            ~ sim_time - How long should the framework run?
%                                       -> 2 - time in seconds
%
%                            ~ metrics - If "compare" specified what needs to be compared?
%                                       -> "distance" - Distance to obstacle 
%                                       -> "time" - Time taken by each planner to execute 
%                                       -> "path" - Path comparisons
%                                  NOTE1:- This needs to be in an array form ["distance", "time", "path"]
%                                  NOTE2:- If "simulation" is chosen, pass an empty array. e.g []
%                              
%
% @ Usage					: MpFramework('simulation','pointmass_v',["rrt"],'Highway', [], 10, [])
%                                   
%
%
%
%@Revision					:       12/07/18                                                                                      
%***************************************************************************

%%
function MpFramework(toDo, model, planner, environment, options, sim_time, metrics)

    planner(1)
% CHECK WHAT THE USER WANTS
   switch (toDo)
       case "compare"
           compare(model,planner,environment, options, sim_time, metrics)
       case "simulation"
           run(model,planner(1),environment, options, sim_time, 0)
       otherwise
           disp('Invalid Argument 1: "compare" and "simulation" are valid')
   end

end

