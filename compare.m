function compare(Vmodel,Planner,Environment,options,SimTime, metrics)    

    %close all windows but do not clear workspace otherwise we loose all params.
    close all;
    
    Ts = 0.02; %this is the sampling time....maybe we can put this on run params?

    %% Chose simulation environment and initialize
    init(Environment,options);
    
    %% Chose vehicle model
    switch lower(Vmodel)
        case 'pointmass_v' 
                pointmass_V(Ts);
        case 'pointmass_j' 
                pointmass_J(Ts);
        case 'dubin' 
                Dubin(Ts);
        otherwise
            disp('Vehicle model not found in database')
    end
    timer_index = 1;
    %%Run all the Planners
    for i=1:1:numel(Planner)
        switch (Planner(i))
            case 'mpc'
                P1_start = tic();
                MpcPlanner(SimTime);
                time_array(timer_index) = toc(P1_start)
                timer_index = timer_index + 1;
            case 'rrt'
                P2_start = tic();
                RRTPlanner(SimTime);
                time_array(timer_index) = toc(P2_start)
                timer_index = timer_index + 1;
            case 'astar'
                P3_start = tic();
                AstarPlanner(SimTime);
                time_array(timer_index) = toc(P3_start)
                timer_index = timer_index + 1;
            otherwise
                disp('Planner not found in database')
        end
    end
    
    %%Perform the Metrics
    for i=1:1:numel(metrics)
            switch (metrics(i))
                case "path"
                    Legend = ["Obstacle", "Target"];
                    figure();hold on
                    for j=1:1:numel(Planner)
                        plotTrajectory(Planner(j), j);
                    end
                    Legend = [Legend Planner];
                    legend(Legend)
                    hold off
                    %legend show;
                    
                    
                case "time"
                    figure(2)
                    Legend = Planner;
                    b_graph = bar(time_array)
                    color = {[1 0 0] [0 1 0] [0 0 1] [0 1 1]};
                    b_graph.FaceColor = 'flat';
                    for j=1:1:numel(Planner)
                        b_graph.CData(j,:) = cell2mat(color(j));
                    end
                    text(1:length(time_array),time_array,num2str(time_array'),'vert','bottom','horiz','center'); 
                    %text(1:length(Planner),Planner,Planner','vert','top','horiz','center');
                    ylabel('Time(s)')
                    xlabel('Planner')
                    legend(b_graph, Legend)
                    legend show
                    title('Execution time comparision')
                    box off
                    
                    
                case "distance" %Calculate the euclideans distance
                    Legend = Planner;
                    figure();hold on
                    for j=1:1:numel(Planner)
                        distanceMetric(Planner(j), j);
                    end
                    legend(Legend)
                    hold off
                    
                    
                otherwise
                    disp("Invalid metrics")
            end
    end

    delete *.mat
end