function plotTrajectory()
    load('environment.mat');
    load('result.mat');
    
    
    figure(1)
    plot(obstacle.X,obstacle.Y, 'rx', 'LineWidth',2);
    rectangle('Position',[obstacle.rrX,obstacle.rrY,obstacle.Length,obstacle.Width],'EdgeColor','r');
    hold on
    plot(goal(1),goal(2), 'rx', 'LineWidth',2);
    hold on
    plot(ydata(1, :), ydata(2, :), 'y', 'LineWidth',2);
    %plot(x1data, y1data, 'y', 'LineWidth',2);
    hold on;
    legend('obstacle', 'target')
    title('Trajectory Comparision')
end

