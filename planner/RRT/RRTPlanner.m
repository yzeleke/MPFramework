% � Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html
function [ydata] = RRTPlanner(simT)
load('environment.mat');
    %load vehilce model
load('model.mat');

rrt_start = tic();
Tsim = simT;
%map=im2bw(imread('map3.bmp')); % input map read from a bmp file. for new maps write the file name here
% source=[10 10]; % source position in Y, X format
% goal=[490 490]; % goal position in Y, X format
resolution_x = 0.1; % I changed this resolution from 0.1 to 0.01 to smoothen the RRT path
resolution_y = 0.1; % I changed this resolution from 0.1 to 0.01 to smoothen the RRT path
upper_bound_x = min(100, upper_bound_x);%ddd
source = x0(1:2,1)';
target = goal(1:2,1)';

[map, source, target] = gridize(obstacle,resolution_x, resolution_y, lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y, source, target);



stepsize=20; % size of each step of the RRT
disTh=20; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
display=false; % display of RRT generation

%%%%% parameters end here %%%%%

tic;
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(target,map), error('goal lies on an obstacle or outside map'); end
% if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
RRTree=double([source -1]); % RRT rooted at the source, representation node and parent index
failedAttempts=0;
counter=0;
pathFound=false;
while failedAttempts<=maxFailedAttempts  % loop to grow RRTs
    if rand < 0.5, 
        sample=rand(1,2) .* size(map); % random sample
    else
        sample=target; % sample taken as goal to bias tree generation to goal
    end
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % find closest as per the function in the metric node to the sample
    closestNode = RRTree(I(1),1:2);
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts=failedAttempts+1;
        continue;
    end
    if distanceCost(newPoint,target)<disTh, pathFound=true;break; end % goal reached
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end 
    RRTree=[RRTree;newPoint I(1)]; % add node
    failedAttempts=0;
    if display, 
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)]);
        counter=counter+1;M(counter)=getframe;
    end
end
if display && pathFound 
    line([closestNode(2);target(2)],[closestNode(1);target(1)]);
    counter=counter+1;M(counter)=getframe;
end
if display 
    disp('click/press any key');
    waitforbuttonpress; 
end
if ~pathFound, error('no path found. maximum attempts reached'); end
path=[target];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength); 
%imshow(map);%rectangle('position',[1 1 size(map)-1],'edgecolor','k');
%line(path(:,2),path(:,1));
ydata = remap(path,resolution_x, resolution_y, lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y);

rrt_time = toc(rrt_start);
ydata = ydata.'; % You need to transpose it so that plotResult function can work..
%ydata = fliplr(ydata);

save('results/resultRRT.mat');