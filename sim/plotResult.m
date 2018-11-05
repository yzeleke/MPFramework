%%
%--------------------------------------------------------------------------
%										plotResults.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke                                            	        
% @file_name				 : 		 plotResult.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :    Plot results/ simulation
%
% @ Usage					:   plotResults(data) data is the
%                                   name of .mat file    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************


function plotResult(data)
    %% The simulation results are identical to those using mpcmove.
    load(data);
    
    t = 0:Ts:Tsim;
    figure;

    
    f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,1);

%     subplot(2,2,1)
%     plot(t,yMPCMOVE(2,:),'--*');
%     title('Plant Output y_{pose}');
%     grid on;
%     legend('mpcmove')

   


    %% Plot results 
    %video
    % initialize
    vidObj = VideoWriter('passing');
    vidObj.FrameRate = 10;
    open(vidObj);
    speed = 2;     % data sample speed

    % create an animation
    hFig = figure(4);
    figure(4);

    for i = 1:speed: length(yMPCMOVE)    % n : length(t)

    % drawing code
       x0 = [yMPCMOVE(1,i),yMPCMOVE(2,i)];
       f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,4); 
    % write each frame to the file
       currFrame = getframe(hFig);
       writeVideo(vidObj,currFrame);
       clf;
    end

    % close the file
    close(vidObj);
    close all
    
end