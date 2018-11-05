function f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,k)
% Create figure
f = figure(k);

% Plot the Ego vehicle.
carLength = 5;
carWidth = 2;
X0 = x0(1);
Y0 = x0(2);
plot(X0,Y0,'gx'); hold on; grid on;
rectangle('Position',[X0-carLength/2,Y0 - carWidth/2,carLength,carWidth],'EdgeColor','r');

set(gca,'Color','k')

plot(goal(1),goal(2),'rx')
rectangle('Position',[X0-carLength/2,Y0 - carWidth/2,carLength,carWidth],'EdgeColor','r');

% Plot the static obstacle.
plot(obstacle.X,obstacle.Y,'rx');
rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');

% Plot the safe zone around obstacle.
%rectangle('Position',[obstacle.rrSafeX,obstacle.rrSafeY,...
%    (obstacle.safeDistanceX)*2,(obstacle.safeDistanceY)*2],...
%    'LineStyle','--','EdgeColor','r');

% Plot the lanes.
X = [0;50;100];
Y = [27;27;27];
line(X,Y,'LineStyle','--','Color','w' );
Y = [23;23;23];
line(X,Y,'LineStyle','--','Color','w' );
Y = [19;19;19];
line(X,Y,'LineStyle','--','Color','w' );
Y = [15;15;15];
line(X,Y,'LineStyle','-','Color','y','LineWidth',2 );
Y = [14;14;14];
line(X,Y,'LineStyle','-','Color','y','LineWidth',2 );
Y = [10;10;10];
line(X,Y,'LineStyle','--','Color','w' );
Y = [6;6;6];
line(X,Y,'LineStyle','--','Color','w' );
Y = [2;2;2];
line(X,Y,'LineStyle','--','Color','w' );
X = [0;50;100];
Y = [-2;-2;-2];
line(X,Y,'LineStyle','--','Color','w' );
Y = [-6;-6;-6];
line(X,Y,'LineStyle','--','Color','w' );
Y = [-10;-10;-10];
line(X,Y,'LineStyle','--','Color','w' );
rectangle('Position',[0,-30,100,20],'FaceColor',[0.5 0.5 0.5],'EdgeColor','b');

% Reset the axis.
axis([0 100 -30 30]);%-laneWidth*lanes/2 laneWidth*lanes/2]);
xlabel('X');
ylabel('Y');
title('Obstacle Avoidance Maneuver');


