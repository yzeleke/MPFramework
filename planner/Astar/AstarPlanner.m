function AstarPlanner()
    %Load environment and vehilce model
    load('environment.mat');
    load('model.mat');
    
    startX = x0(1);
    startY = x0(2);
    targetX = goal(1);
    targetY = goal(2);
    targetTheta = 0;
    Theta = 72;
    Theta_Res = 5;
    BOT_L = 34;
    %BOT_W = 20;
    BOT_M_ALPHA = 30;
    PRIORITY_OBSTACLE_NEAR = 10;
    PRIORITY_MOVEMENT = 5;

    xMax = 1000;
    yMax = upper_bound_y;

     

    
    %build space we are navigating
    
    %assign cost to each map position, runDijkstra in Compare.cpp
    cells = repmat(mapCell(10000, 0, 0), xMax, yMax);
    cells(targetX, targetY).cost2d = 0;
    cells(targetX, targetY).dx = targetX;
    cells(targetX, targetY).dy = targetY;
    frontier = PQ2(); %priority_queue<State, vector<State>, compare2dSignature> frontier(&compare2d);
    
    push(frontier, cells(targetX, targetY), getKey(cells(1, 1), targetX, targetY, targetX, targetY));

    while isEmpty(frontier) == 0
		current=pop(frontier);
        
        x=current.dx;
		y=current.dy;
        for i=-1:1:1
            for j=-1:1:1
                if(x+i<1 || x+i>=xMax || y+j<1 || y+j>=yMax)
					continue;
                elseif((i==0 && j==0) || cells(x, y).closed==1)
					continue;
                elseif(cells(x+i, y+j).cost2d>cells(x, y).cost2d+sqrt(i*i+j*j))
					cells(x+i, y+j).cost2d=cells(x, y).cost2d+floor(sqrt(i*i+j*j));
					tempstate.dx=current.dx+i;
					tempstate.dy=current.dy+j;
					tempstate.cost2d=cells(x+i, y+j).cost2d;
					push(frontier, tempstate, getKey(cells(tempstate.dx, tempstate.dy), targetX, targetY, targetX, targetY));
                end
            end
        end
    end
    %end runDjikstra
% 
% 	map.initCollisionChecker();
% 	map.find_near_obs();
% 
    pq = PQ2(); %priority_queue<State, vector<State>, Compare> pq;
    cells(startX, startY).cost3d = 0; % 	start.cost3d=0;
 	push(pq, cells(startX, startY), 0);
    
    previous = repmat(mapCell(10000, 0, 0), xMax, yMax);

    while(isEmpty(pq) == 0)
		current=pop(pq);

        if(abs(current.x-targetX)<=1 & abs(current.y-targetY)<=1 & abs(current.theta-targetTheta)<=5)
			disp("Reached target.");
			current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(map.obs_dist_max-1) + fabs(current.theta)/BOT_M_ALPHA +1; 
				
            while(current.x~=start.x || current.y~=start.y || current.theta~=start.theta)
				current.velocity=VELOCITY_MAX/current.change;
				Dummy=previous(current.x, current.y, current.theta);
				Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.theta-current.theta)/(2.0*BOT_M_ALPHA)+PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(map.obs_dist_max-1)+fabs(Dummy.theta)/BOT_M_ALPHA+1;
				current=Dummy;
            end
			break;
        end

        %next=current.getNextStates();
        %begin getNextStates
        next = repmat(mapCell(0, 0, 0), 3);
        d=40;
        stateCount = 1;

        for alpha=-BOT_M_ALPHA : BOT_M_ALPHA : BOT_M_ALPHA+0.001
            beta=d*tan(alpha*pi/180)/BOT_L;
            if(abs(beta)<0.001)
                n.x=x+d*cos(current.theta*2.0*pi/Theta);
                n.y=y+d*sin(current.theta*2.0*pi/Theta);
                n.theta=current.theta;
            else
                r=BOT_L/tan(alpha*pi/180);
                next(stateCount).x=x+r*sin(current.theta*2.0*pi/Theta+beta)-r*sin(current.theta*2.0*pi/Theta);
                next(stateCount).y=y-r*cos(current.theta*2.0*pi/Theta+beta)+r*cos(current.theta*2.0*pi/Theta);
                if(current.theta + beta*180/pi/Theta_Res>0)
        			next(stateCount).theta=fmod(current.theta + beta*180/pi/Theta_Res,Theta);
                else
                    next(stateCount).theta=current.theta + beta*180/pi/Theta_Res+Theta;
                end
            end
%            n.gx=n.x/Grid_Res;
%            n.gy=n.y/Grid_Res;
%            n.gtheta=n.theta+0.01;
%            next.push_back(n);
        end

	% cout<<"getNextStates() called from "<<x<<","<<y<<","<<theta<<endl;
	% for(int i=0;i<3;i++) cout<<next[i].x<<","<<next[i].y<<","<<next[i].theta<<"  "; cout<<endl;
    %end getNextStates

        for i=1:1:3
            if next(i).x ~= obstacle.X & next(i).y ~= obstacle.Y %if(~map.checkCollision(next(i)))
                if(i==2)
					next(i).cost3d=current.cost3d+5;
				else
					next(i).cost3d=current.cost3d+7;
                end
				current.next=next(i);
				next(i).previousX=current.x;
                next(i).previousY=current.y;
				pq.push(next(i));
                previous(next(i).x, next(i).y, next(i).theta)=current;
            end
        end
    end
    result = previous;
    save(result);
end