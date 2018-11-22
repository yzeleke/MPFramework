function AstarPlanner(startX, startY, targetX, targetY)
    xMax = targetX;
    yMax = targetY;
    %Load environment and vehilce model
%     load('environment.mat');
%     load('model.mat');
    
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
% 	priority_queue<State, vector<State>, Compare> pq;
% 	start.cost3d=0;
% 	pq.push(start);
% 
% 	GUI display(800, 800);
% 	display.drawObs(map);
% 	display.drawCar(start);
% 	display.drawCar(target);
% 
% 	int vis[GX][GY][Theta];
% 	memset(vis, 0, sizeof(int)*GX*GY*Theta);
% 
% 	int iter=0;
% 	while(pq.size()>0)
% 	{
% 		State current=pq.top();
% 		pq.pop();
% 
% 		if(abs(current.gx-target.gx)<=1 && abs(current.gy-target.gy)<=1 && abs(current.gtheta-target.gtheta)<=5){
% 			cout<<"Reached target."<<endl;
% 
% 			State Dummy;
% 			current.change=PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(current))/(float)(map.obs_dist_max-1)+
% 						   fabs(current.theta)/BOT_M_ALPHA+1; 
% 				
% 			while(current.x!=start.x || current.y!=start.y || current.theta!=start.theta){
% 				current.velocity=VELOCITY_MAX/current.change;
% 				display.drawCar(current);
% 			        display.show(2000/current.velocity);//This can be removed while executing the algo
% 				Dummy=previous[current.gx][current.gy][current.gtheta];
% 				Dummy.change=PRIORITY_MOVEMENT*fabs(Dummy.theta-current.theta)/(2.0*BOT_M_ALPHA)+
% 					     PRIORITY_OBSTACLE_NEAR*(map.obs_dist_max-map.nearest_obstacle_distance(Dummy))/(float)(map.obs_dist_max-1)+
% 					     fabs(Dummy.theta)/BOT_M_ALPHA+1;
% 				current=Dummy;
% 			}
% 			break;
% 		}
% 
% 		if(vis[current.gx][current.gy][current.gtheta]){
% 			continue;
% 		}
% 
% 		vis[current.gx][current.gy][current.gtheta]=1;
% 
% 		vector<State> next=current.getNextStates();
% 
% 		for(int i=0;i<next.size();i++){
% 			//display.drawCar(next[i]);
% 			if(!map.checkCollision(next[i])){
% 
% 
% 				if(!vis[next[i].gx][next[i].gy][next[i].gtheta]){
% 					//display.drawCar(next[i]);
% 					current.next=&(next[i]);
% 					next[i].previous=&(current);
% 					if(i==1)
% 						next[i].cost3d=current.cost3d+5;
% 					else
% 						next[i].cost3d=current.cost3d+7;
% 					//next[i].cost3d=current.cost3d+1;
% 					pq.push(next[i]);
% 
% 					previous[next[i].gx][next[i].gy][next[i].gtheta]=current;
% 				}
% 			}
% 		}
% 	}
% 	cout<<"Done."<<endl;
% 	display.show(0);
% 
% 	return;
end