%%%%%%%%%%%%%%%%%%%%%%%%%%
% ppp_struct(x0,xf,tf)
% x0: Struct of initial conditions
% xf: Struct of final conditions
% tf: Duration of path in seconds
%%%%%%%%%%%%%%%%%%%%%%%%%%
function [x_traj,y_traj] =ppp_struct(x0, xf, tf)


% Get x trajectory
D = ppp_points(x0.x, x0.xv, x0.xa, xf.x, xf.xv, xf.xa, tf);
D = flipud(D); %ppp outputs [c5;c4...c0]
x_traj = @(t) D(1) + D(2)*t + D(3)*t.^2 + D(4)*t.^3 + D(5)*t.^4 + D(6)*t.^5;

% Get y trajectory
D = ppp_points(x0.y, x0.yv, x0.ya, xf.y, xf.yv, xf.ya, tf);
D = flipud(D); %ppp outputs [c5;c4...c0]
y_traj = @(t) D(1) + D(2)*t + D(3)*t.^2 + D(4)*t.^3 + D(5)*t.^4 + D(6)*t.^5;



