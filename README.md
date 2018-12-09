# MPFramework
This repository contains a framework for simulating and deploying various motion planning algorithm.

# Software Requirements
Matlab 2017 or before
MPC toolbox

# Arguments
                            ~ toDo - What should the framework do?
                                       -> "compare" - use this as input to compare planners
                                       -> "simulation" - use this to simulate one of the planner

                            ~ model - What model should be used to compare or simulate planners
                                       -> "pointmass_v" - pointmass model that doesn't include jerk 
                                       -> "pointmass_j" - pointmass model that includes jerk and velocity
                                       -> "dubin" - dubins model

                            ~ planner - What planner/s to compare or simulate?
                                       -> "mpc" - MPC planner 
                                       -> "rrt" - RRT planner
                                       -> "astar" - Astar planner
                                  NOTE:- This needs to be in an array form ["mpc", "rrt", "astar"] 

                            ~ environment - What environment the planner and model should be ran in?
                                       -> "H" - highway environment 
                                       -> "P" - Parking Lot  (Coming soon)

                            ~ options - Options to modify Environment?
                                       -> pass in empty array for now... e.g []

                            ~ sim_time - How long should the framework run?
                                       -> 2 - time in seconds

                            ~ metrics - If "compare" specified what needs to be compared?
                                       -> "distance" - Distance to obstacle 
                                       -> "time" - Time taken by each planner to execute 
                                       -> "path" - Path comparisons
                                  NOTE1:- This needs to be in an array form ["distance", "time", "path"]
                                  NOTE2:- If "simulation" is chosen, pass an empty array. e.g []
# How to run
eg,
   For simulation: 
           MpFramework('simulation','pointmass_v',["rrt"],'Highway', [], 10, [])
   
   For comparision: 
           MpFramework('compare','pointmass_v',["rrt", "astar"],'Highway', [], 10, ["path", "time"])
     
