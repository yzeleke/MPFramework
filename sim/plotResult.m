%%
%--------------------------------------------------------------------------
%										plotResults.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke                                            	        
% @file_name				 : 		plotResult.m														  
% @ Date                     :      11/02/18                                                     
% @ Discription				 :      Plot results/ simulation
%
% @ Usage					 :      plotResults(data) data is the
%                                   name of .mat file    																						  
% @Revision					 :      11/5/18                                                                                      
%***************************************************************************


function plotResult()
    %% The simulation results are identical to those using mpcmove.
    load('init.mat');
    load('environment.mat');
    load('result.mat');
    
    t = 0:Ts:Tsim;
    figure;

    
    %f = obstaclePlotInitialCondition(x0,obstacle,laneWidth,lanes,goal,carLength,carWidth);

%     subplot(2,2,1)
%     plot(t,yMPCMOVE(2,:),'--*');
%     title('Plant Output y_{pose}');
%     grid on;
%     legend('mpcmove')

   


    %% Plot results 
    %video
    % initialize
    switch model
        case 'pointmass_v' %can you look up a way to use both 'pointmass' and 'Pointmass'
              vidObj = VideoWriter('passing_Input_v');
         case 'pointmass_j' %can you look up a way to use both 'pointmass' and 'Pointmass'
              vidObj = VideoWriter('pointmass_Input_jerk');
                
        case 'Dubin' %can you look up a way to use both 'pointmass' and 'Pointmass'
                 vidObj = VideoWriter('Dubin');
        otherwise
            disp('Error: plotResult()...Model name can not be found?')
    end
   
    vidObj.FrameRate = 60;
    open(vidObj);
    speed = 2;     % data sample speed

    % create an animation
    hFig = figure(1);
    figure(1);

    for i = 1:speed: length(ydata)    % n : length(t)

    
    % drawing code
       x0 = [ydata(1,i),ydata(2,i)];
       f = plotHighway(x0,obstacle,laneWidth,lanes,goal,carLength,carWidth);
    % write each frame to the file
       currFrame = getframe(hFig);
       writeVideo(vidObj,currFrame);
       clf;
    end

    % close the file
    close(vidObj);
    close all
    
end