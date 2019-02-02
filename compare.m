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
                MpcPlanner(SimTime);
                load('results/resultMPC.mat');
                time_array(timer_index) = mpc_time;
                timer_index = timer_index + 1;
            case 'rrt'
                RRTPlanner(SimTime);
                load('results/resultRRT.mat');
                time_array(timer_index) = rrt_time;
                timer_index = timer_index + 1;
            case 'astar'
                %P3_start = tic();
                AstarPlanner(SimTime);
                load('results/resultAstar.mat');
                time_array(timer_index) = astar_time;
                timer_index = timer_index + 1;
            case 'ppp'
                PolynomialPathPlanner(SimTime);
                load('results/resultPPP.mat');
                time_array(timer_index) = ppp_time;
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
                    figure(1);hold on
                    for j=1:1:numel(Planner)
                        plotTrajectory(Planner(j), j);
                    end
                    Legend = [Legend Planner];
                    legend(Legend)
                    hold off
                    %legend show;
                    
                    
                case "time"
                    figure(2);hold on
                    names = [""]
                    for j=1:1:numel(Planner)
                        names(j) = Planner(j);
                    end
                    %names = {Planner(1),'mpc','astar','ppp'};
                    b_graph = bar(time_array)
                    color = {[1 0 0] [0 1 0] [0 0 1] [0 1 1]};
                    b_graph.FaceColor = 'flat';
                    for j=1:1:numel(Planner)
                        b_graph.CData(j,:) = cell2mat(color(j));
                        %hold on
                    end
                    text(1:length(time_array),time_array,num2str(time_array'),'vert','bottom','horiz','center'); 
                    %text(1:length(Planner),Planner,Planner','vert','top','horiz','center');
                    ylabel('Time(s)')
                    xlabel('Planner')
                    legend(b_graph, names)
                    legend show
                    title('Execution time comparision')
                    box off
                    hold off
                    
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
