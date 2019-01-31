%%%%%%%%%%%%%%%%%%%%%%%%%%
% gen_pp_traj(x0,xf,tf)
% x0: Vertex with initial conditions
% xf: Vertex with final conditions
% tf: Duration of path in seconds
% time_res: Time resolution
%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_traj,y_traj, arc_length] =gen_pp_traj(x0, xf, tf, time_res)


% Get x trajectory
D = ppp_points(x0.x, x0.xv, x0.xa, xf.x, xf.xv, xf.xa, tf);
D = flipud(D); %ppp outputs [c5;c4...c0]
x_traj     = @(t) D(1) + D(2)*t + D(3)*t.^2 + D(4)*t.^3 + D(5)*t.^4 + D(6)*t.^5;
x_traj_dot = @(t) D(2) + 2*D(3)*t.^1 + 3*D(4)*t.^2 + 4*D(5)*t.^3 + 5*D(6)*t.^4;

% Get y trajectory
D = ppp_points(x0.y, x0.yv, x0.ya, xf.y, xf.yv, xf.ya, tf);
D = flipud(D); %ppp outputs [c5;c4...c0]
y_traj     = @(t) D(1) + D(2)*t + D(3)*t.^2 + D(4)*t.^3 + D(5)*t.^4 + D(6)*t.^5;
y_traj_dot = @(t) D(2) + 2*D(3)*t.^1 + 3*D(4)*t.^2 + 4*D(5)*t.^3 + 5*D(6)*t.^4;

% Calculate the arc length
l=0;
for(t=0:time_res:tf)
   l = l + sqrt(x_traj_dot(t)^2 + y_traj_dot(t)^2)*time_res; 
end
arc_length = l;

