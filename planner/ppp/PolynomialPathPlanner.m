%%
%--------------------------------------------------------------------------
%							 PolynomialPathPlanner.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 	David Kooi	
% @file_name				 :  PolynomialPathPlanner.m														  
% @ Date                     : 	11/02/18                                                     
% @ Description				 :    
%                                
%                               
% @ Usage					:   MpcPlanner(model) model is the
%                                   name of .mat file    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function PolynomialPathPlanner(Tsim)
    % Load environment
    load('environment.mat');
    %load vehilce model
    load('model.mat');


    % Number of vertices
    x_max = 30;
    y_max = 10


    % Setup obstacles
    obs_set(1).x = [obstacle.flSafeX, obstacle.frSafeX, obstacle.rlSafeX, obstacle.rrSafeX];
    obs_set(1).y = [obstacle.flSafeY, obstacle.frSafeY, obstacle.rlSafeY, obstacle.rrSafeY];

     
    % Goal and start vertices
    goal        = Vertex;
    goal.g      = inf;
    goal.rhs    = 0;
    goal.x      = goal(1);
    goal.y      = goal(2);
    goal.xv     = 1;
    goal.yv     = 0;
    goal.xa     = 0;
    goal.ya     = 0;

    start       = Vertex;
    start.g     = inf;
    start.rhs   = inf;
    start.x     = x0(1);
    start.y     = x0(2);
    start.xv    = 1;
    start.yv    = 0;
    start.xa    = 0;
    start.ya    = 0;
    start.final_v = start

    U = PQ2(); % Create priority queue 


    % Initialize all vertices
    for i=1:x_max    
       for(j=1:y_max)
           v = Vertex;
           
           % Distance metrics
           v.g   = inf;
           v.rhs = inf;
           
           % State values
           v.x = i;
           v.y = j;   
           
           v.xv = 0;
           v.yv = 0;
           
           v.xa = 0;
           v.ya = 0;
           

           if(i == goal.x && j == goal.y)
            v_list(i,j) = goal;   
           elseif(i == start.x && j == start.y)
            v_list(i,j) = start;
           else
            v_list(i,j) = v;
           end        
       end
    end 

    % Add adjacent vertices assuming 8-connected graph
    % Adjacent vertices will be predecesors and successors
    % ...But that could change
    for(i=1:x_max)
        for(j=1:y_max)

                v = v_list(i,j);
                
                % Collect adjacent vertices
                for(k=1:x_max)
                    for(g=1:y_max)
                        if( (v.x+1 == k && abs(v.y-g) <= 1) || ...
                            (v.x-1 == k && abs(v.y-g) <= 1) || ...
                            (v.y+1 == g && abs(v.x-k) == 0) || ...
                            (v.y-1 == g && abs(v.x-k) == 0) )
                            v.pred_list = [v.pred_list v_list(k,g)];
                            v.succ_list = [v.succ_list v_list(k,g)];
                        end                 
                    end
                end

                % Book-keep number of predecesors and successors
                v.num_pred = length(v.pred_list);
                v.num_succ = length(v.succ_list); 
        end
    end


tic
% Begin algorithm
U.push(v_list(goal.x, goal.y), calc_key(goal, start));

while(U.min_key() < calc_key(start, start) || start.rhs ~= start.g)
    curr_v = U.pop();
    if(curr_v.g > curr_v.rhs)
        curr_v.g = curr_v.rhs;
        update_verticies(curr_v, goal, U, obs_set, start);
    else
        curr_v.g = inf;
        update_verticies(curr_v, goal, U, obs_set, start);      
    end
end


% Trace back shortest path
x_traj = [];
y_traj = [];
time_res  = Vertex().time_res;
path_time = Vertex().path_time;
t = 0:time_res:path_time;


curr_v = goal;
visited = [goal]; % Keep track of the visited vertices
while(curr_v ~= start)
    prev_v = Vertex;
    prev_v.g = Inf;
    for(i=1:curr_v.num_pred) % Iterate through curr_v's predecessor list
        v = curr_v.pred_list(i); 
        if(v.g < prev_v.g && sum(ismember(visited, v)==0)) % Choose the
           if(v.next == curr_v)                            % predecessor 
                prev_v = v;                                % with lowest g value 
           end                                             % and make sure it has not been visited.
        end
    end
    visited = [visited prev_v];
    %x_traj = [x_traj prev_v.x_traj(t)];
    %y_traj = [y_traj prev_v.y_traj(t)];
    %scatter(x_traj,y_traj,20,'filled');
    %hold on;
    
    curr_v = prev_v;
end
toc

ydata = [x_traj; y_traj]; 
save('results/resultsPPP.mat')

end












e