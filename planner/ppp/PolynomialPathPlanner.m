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

    x_max = 100;
    y_max = 100;

    step_size_x = 10;
    step_size_y = 1;


    % Setup obstacles
    obs_set(1).x = [obstacle.flSafeX, obstacle.frSafeX, obstacle.rlSafeX, obstacle.rrSafeX];
    obs_set(1).y = [obstacle.flSafeY, obstacle.frSafeY, obstacle.rlSafeY, obstacle.rrSafeY];

     
    % Goal and start vertices
    target        = Vertex;
    target.g      = inf;
    target.rhs    = 0;
    target.x      = goal(1);
    target.y      = goal(2);
    target.xv     = 1;
    target.yv     = 0;
    target.xa     = 0;
    target.ya     = 0;

    start       = Vertex;
    start.g     = inf;
    start.rhs   = inf;
    start.x     = x0(1);
    start.y     = x0(2);
    start.xv    = 1;
    start.yv    = 0;
    start.xa    = 0;
    start.ya    = 0;
    start.final_v = start;

    U = ppp_PQ2(); % Create priority queue 


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
           

           if(i == target.x && j == target.y)
            v_list(i,j) = target;   
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
               

                % Tuples of nearest neighbors
                nn{1} = [i+step_size_x, j];
                nn{2} = [i+step_size_x, j+step_size_y];
                nn{3} = [i, j+step_size_y];
                nn{4} = [i-step_size_x,j+step_size_y]; 
                nn{5} = [i-step_size_x, j];
                nn{6} = [i-step_size_x, j-step_size_y];
                nn{7} = [i, j-step_size_y];
                nn{8} = [i+step_size_x, j-step_size_y];

                for(k=1:8)
                    tup = nn{k};

                    % Bound x coordinates
                    if(tup(1) > x_max)
                        tup(1) = x_max;
                    end

                    if(tup(1) < 1)
                        tup(1) = 1;
                    end

                    % Bound Y coordinates
                    if(tup(2) > y_max)
                        tup(2) = y_max;
                    end

                    if(tup(2) < 1)
                        tup(2) = 1;
                    end


                    this_x = tup(1);
                    this_y = tup(2);
                    v.pred_list = [v.pred_list v_list(this_x,this_y)];
                    v.succ_list = [v.succ_list v_list(this_x,this_y)];

                end
                
                % Collect adjacent vertices
%                for(k=1:x_max)
%                    for(g=1:y_max)
%                        if( (v.x+1 == k && abs(v.y-g) <= 1) || ...
%                            (v.x-1 == k && abs(v.y-g) <= 1) || ...
%                            (v.y+1 == g && abs(v.x-k) == 0) || ...
%                            (v.y-1 == g && abs(v.x-k) == 0) )
%                            v.pred_list = [v.pred_list v_list(k,g)];
%                            v.succ_list = [v.succ_list v_list(k,g)];
%                        end                 
%                    end
%                end

                % Book-keep number of predecesors and successors
                v.num_pred = length(v.pred_list);
                v.num_succ = length(v.succ_list); 
        end
    end


tic
% Begin algorithm
U.push(v_list(target.x, target.y), calc_key(target, start));

while(U.min_key() < calc_key(start, start) || start.rhs ~= start.g)
    curr_v = U.pop();
    if(curr_v.g > curr_v.rhs)
        curr_v.g = curr_v.rhs;
        update_verticies(curr_v, target, U, obs_set, start);
    else
        curr_v.g = inf;
        update_verticies(curr_v, target, U, obs_set, start);      
    end
end


% Trace back shortest path
x_traj = [];
y_traj = [];
time_res  = Vertex().time_res;
path_time = Vertex().path_time;
t = 0:time_res:path_time;


curr_v = start;
while(curr_v ~= target)
     x_traj = [x_traj curr_v.x_traj(t)];
     y_traj = [y_traj curr_v.y_traj(t)];
     scatter(x_traj,y_traj,20,'filled');
     hold on;
     
     curr_v = curr_v.next;
    
end


toc

ydata = [x_traj; y_traj]; 
save('results/resultsPPP.mat')

end



