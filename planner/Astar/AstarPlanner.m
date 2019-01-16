function AstarPlanner(simT)
    %Load environment and vehilce model
     load('environment.mat');
     load('model.mat');
    
    disp("hi");
    Tsim = simT; 
    startX = x0(1); %+40
    startY = x0(2);
    targetX = goal(1);
    targetY = goal(2);
    %targetTheta = 0;
%     Theta = 72;
%     Theta_Res = 5;
    BOT_L = 2;
    %BOT_W = 20;
    BOT_M_ALPHA = 30;
%     PRIORITY_OBSTACLE_NEAR = 10;
%     PRIORITY_MOVEMENT = 5;
    
    %step size
    d=1;

    xMax = 100;%upper_bound_x;
    yMax = upper_bound_y;
    
    %prevents x form begin inf in testing
    if(xMax > 100)
        xMax = 100;
    end
    
    fprintf("Target is (%d, %d)\n", targetX, targetY);

    pq = PQ2(); %priority_queue<State, vector<State>, Compare> pq;
    startCell = mapCell(0, 0, 0); % 	start.cost3d=0;
    startCell.x = startX;
    startCell.y = startY;
    startCell.theta = 0;
 	push(pq, startCell, 0);
    fprintf("start (%f, %f), obstacle (%f, %f)\n", startX, startY, obstacle.X, obstacle.Y);
    
    previous = repmat(mapCell(10000, 0, 0), xMax, yMax, 360);

    while(isEmpty(pq) == 0)
		current=pop(pq);
        if(floor(current.x) == targetX)
            if(floor(current.y) == targetY)% & abs(current.theta-targetTheta)<=5)
                fprintf("Reached target.\n");
                %imshow(map);
                %current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(map.obs_dist_max-1) + fabs(current.theta)/BOT_M_ALPHA +1; 
                x1data(1) = current.x;
                y1data(1) = current.y;
			
                while(current.x~=startX || current.y~=startY)% || current.theta~=start.theta)
				%current.velocity=VELOCITY_MAX/current.change;
                    Dummy=previous(floor(current.x), floor(current.y), floor(current.theta)+180);
    				%Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.theta-current.theta)/(2.0*BOT_M_ALPHA)+PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(map.obs_dist_max-1)+fabs(Dummy.theta)/BOT_M_ALPHA+1;
        			current=Dummy;
                    x1data(end+1) = current.x;
                    y1data(end+1) = current.y;
                end
                fprintf("done\n");
                break;
            end
        end

        %next=current.getNextStates();
        %begin getNextStates
        next = repmat(mapCell(0, 0, 0), 3, 1);
%         stateCount = 1;

%         for alpha=-BOT_M_ALPHA : BOT_M_ALPHA : BOT_M_ALPHA+0.001
        for stateCount = 1:1:3
%             beta=d*tan(alpha*pi/180)/BOT_L;
            if(stateCount == 2)%abs(beta)<0.001)
                next(stateCount).x=current.x+d;
%                 next(stateCount).x=current.x+d*cos(current.theta*2.0*pi/Theta);
                %fprintf("updated %f", next(stateCount).x);
                next(stateCount).y=current.y;
                next(stateCount).theta = 2;
%                 next(stateCount).y=current.y+d*sin(current.theta*2.0*pi/Theta);
%                 next(stateCount).theta=current.theta;
            else
%                 r=BOT_L/tan(alpha*pi/180);
                next(stateCount).x=current.x;
%                 next(stateCount).x=current.x+r*sin(current.theta*2.0*pi/Theta+beta)-r*sin(current.theta*2.0*pi/Theta);
                next(stateCount).y=current.y+(stateCount-2);
                next(stateCount).theta = stateCount;
%                 next(stateCount).y=current.y-r*cos(current.theta*2.0*pi/Theta+beta)+r*cos(current.theta*2.0*pi/Theta);
%                 if(current.theta + beta*180/pi/Theta_Res>0)
%         			next(stateCount).theta=mod(current.theta + beta*180/pi/Theta_Res,Theta);
%                 else
%                     next(stateCount).theta=current.theta + beta*180/pi/Theta_Res+Theta;
%                 end
            end
%             stateCount = stateCount + 1;
%            n.gx=n.x/Grid_Res;
%            n.gy=n.y/Grid_Res;
%            n.gtheta=n.theta+0.01;
%            next.push_back(n);
        end

        for i=1:1:3
            if(next(i).x >= 1 && next(i).x <= xMax && next(i).y >= 1 && next(i).y <= yMax)
                if ~(next(i).x >= obstacle.rlSafeX && next(i).x <= obstacle.flSafeX && next(i).y >= obstacle.frSafeY && next(i).y <= obstacle.flSafeY) %if(~map.checkCollision(next(i)))
                    if(i==2)
            			next(i).cost3d=current.cost3d+1;
                    else
    					next(i).cost3d=current.cost3d+(floor(next(i).y)-targetY);
                    end
                    current.next=next(i);
                    next(i).previousX=current.x;
                    next(i).previousY=current.y;
                    push(pq, next(i), next(i).cost3d + (5*hypot(targetX - floor(next(i).x), floor(targetY - next(i).y))));%pq.push(next(i));
                    previous(floor(next(i).x), floor(next(i).y), floor(next(i).theta)+180)=current;
                end
            end
        end
    end
    ydata = [x1data; y1data];
    save('results/resultAstar.mat');
    fprintf("pq empty\n");
end