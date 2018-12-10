
% Calculate the minimum polynomial cost from one vertex to another.
% We need to discritize the state space of the end vertex
function cost = cost_to_vertex(start_v, end_v, obs_set)

% Itereate through a discretization of the start vertex's state space
deg = 0;
min_cost = inf;
min_cost_vertex = copy(start_v);
for(i=1:8)
    
    % Assume unit velocities for now
    min_cost_vertex.xv = cosd(deg);
    min_cost_vertex.yv = sind(deg);
    deg = deg + 45;
    
    % Also assume unit time
    [x_traj, y_traj, arc_length] = gen_pp_traj(min_cost_vertex, end_v,...
                                               min_cost_vertex.path_time,...
                                               min_cost_vertex.time_res);
    
    t = 0:Vertex().time_res:Vertex().path_time;
    x = x_traj(t);
    y = y_traj(t);
    plot(x,y);
    hold on;
    
    % Check intersection of objs set
    for(i=1:length(obs_set))
        obs = obs_set(i);
        [in,on] = inpolygon(x,y, obs.x,obs.y);
        if(numel(x(in)) > 0 ||...
           numel(x(on)) > 0)
           arc_length=inf;
           break;
        end
    end

    
    if(arc_length < min_cost)
        min_cost = arc_length;
        
        % Save the trajectories for future use
        start_v.x_traj = x_traj;
        start_v.y_traj = y_traj;
        start_v.next   = end_v;
        start_v.xv     = min_cost_vertex.xv;
        start_v.yv     = min_cost_vertex.yv;
    end
    
end
cost = min_cost;
end