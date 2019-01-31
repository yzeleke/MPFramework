clear all;
clc;

% Rectangular obstacle
width = 1; % whatever
height = 1; % whatever...
xCenter = 4; % Wherever...
yCenter = 0; % Wherever...
xLeft = xCenter - width/2;
yBottom = yCenter - height/2;
rectangle('Position', [xLeft, yBottom, width, height], 'EdgeColor', 'b', 'FaceColor', 'r', 'LineWidth', 4);
grid on;
hold on;


% For each potential goal, calculate the trajectory from the intial state
tf   = 5;     % Final time
t = 0:0.1:tf; % Time instances
x0.x = 0;     % Initial position
x0.y = 0;

x0.xv = 1;    % Initial velocity
x0.yv = 0;

x0.xa = 0;    % Initial acceleration
x0.ya = 0;


% Goal points
step = 0.5;
for i=1:10
    goal_points(i).x = 10
    goal_points(i).y = step*i;
    
    goal_points(i).xv = x0.xv;
    goal_points(i).yv = x0.yv;
    
    goal_points(i).xa = x0.xa;
    goal_points(i).ya = x0.ya;
    
end



% For each goal point calculate the polynomial trajectory
for i=1:length(goal_points)(1,1
    [x_traj, y_traj] = ppp_struct(x0, goal_points(i), tf);
    
    % Plot
    plot(x_traj(t), y_traj(t));
    hold on;
    
end
axis([0 10 0 5.5]);


