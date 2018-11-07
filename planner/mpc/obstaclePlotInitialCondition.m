function f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,carLength,carWidth)
    % Create figure
    f = figure(1);


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



    % Plot the lanes.
    X = [0;50;100];
    for i = 0:laneWidth: laneWidth*lanes
        Y = [i;i;i];
        line(X,Y,'LineStyle','--','Color','w' );
    end
  

    % Reset the axis.
    axis([0 100 0 laneWidth*lanes+5]);%-laneWidth*lanes/2 laneWidth*lanes/2]);
    xlabel('X');
    ylabel('Y');
    title('Obstacle Avoidance Maneuver');
end

