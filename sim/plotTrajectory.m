function plotTrajectory(SimTime)
    load('environment.mat');
    
    
    %MpcPlanner(SimTime);
    load('result.mat');
    
    
%     figure(1)
%     plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
%     rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
%     hold on
%     plot(goal(1),goal(2), 'rx', 'LineWidth',2);
%     hold on
%     
%     
%     plot(ydata(1, :), ydata(2, :), 'b', 'LineWidth',2);
%     hold on;
%     RRTPlanner(SimTime);
%     load('result.mat');
%     
%     
%     plot(ydata(1, :), ydata(2, :), 'b', 'LineWidth',2);
%     hold on;
%     plot(ydata(1, :), ydata(2, :), 'r', 'LineWidth',2);
%     hold on;
%     
    AstarPlanner(SimTime);
    load('result.mat');
    figure(1)
    plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
    rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
    hold on
    plot(goal(1),goal(2), 'rx', 'LineWidth',2);
    hold on
    
    %plot(ydata(1, :), ydata(2, :), 'y', 'LineWidth',2);
    plot(x1data, y1data, 'y', 'LineWidth',2);
    hold on;
    
    %legend('obstacle', 'target', 'MPC','RRT', 'Astar')
    legend('obstacle', 'target','Astar')
    title('Trajectory Comparision')
end

