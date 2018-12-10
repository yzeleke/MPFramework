clc;
clear all;

% Settings

x_max = 10; % Number of verticies
y_max = 10;


% Goal and start vertices
goal        = Vertex;
goal.g      = inf;
goal.rhs    = 0;
goal.x      = 10;
goal.y      = 10;
goal.xv     = 1
goal.yv     = 1;
goal.xa     = 0;
goal.ya     = 0;

start       = Vertex;
start.g     = inf;
start.rhs   = inf;
start.x     = 1;
start.y     = 1;
start.xv    = 1;
start.yv    = 1;
start.xa    = 0;
start.ya    = 0;

% Obstacle Set
p1 = Point(7,7); p2 = Point(7,8); p3 = Point(8,8); p4 = Point(8,7);
p5 = Point(4,7); p6 = Point(5,7); p7 = Point(5,8); p8 = Point(4,8);

obs_set(1).x = [p1.x,p2.x,p3.x,p4.x];
obs_set(1).y = [p1.y,p2.y,p3.y,p4.y];
obs_set(2).x = [p5.x, p6.x, p7.x, p8.x];
obs_set(2).y = [p5.y, p6.y, p7.y, p8.y];

U = PQ2(); % Create priority queue 


for i=1:x_max    % Initialize all vertices
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
       

% Create the lattice
% First gather all points
x = [];
y = [];

for(i=1:x_max)
   for(j=1:y_max)
       x = [x i];
       y = [y j];
   end
end
scatter(x,y,100, 'filled');
hold on;
for(i=1:length(obs_set))
    plot(obs_set(i).x, obs_set(i).y);
    hold on;
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
    for(i=1:curr_v.num_pred)
        v = curr_v.pred_list(i);
        if(v.g < prev_v.g && sum(ismember(visited, v)==0))
           if(v.next == curr_v)
                prev_v = v;
           end
        end
    end
    visited = [visited prev_v];
    x_traj = [x_traj prev_v.x_traj(t)];
    y_traj = [y_traj prev_v.y_traj(t)];
    scatter(x_traj,y_traj,20,'filled');
    hold on;
    
    curr_v = prev_v;
end
toc



