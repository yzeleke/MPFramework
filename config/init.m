%%
%--------------------------------------------------------------------------
%										init.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 		Yegeta Zeleke and Harsh Bhakta                                           	        
% @file_name				 : 		 MpcPlanner.m														  
% @ Date                     : 	   11/02/18                                                     
% @ Discription				 :    system initializer
%
% @ Usage					:   init(varargin), i,e, init('highway','len',3...)    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function init(Environment,options)
    

    
    %default car dimension
    carLength = 5;
    carWidth = 2;
  
    
    if ~isempty(options)
        %change the stuff here 
        carLength = 10;
        carWidth = 4;
    end

    switch Environment
        case 'Highway' %can you look up a way to use both 'pointmass' and 'Pointmass'
                init_highway(options) 
        otherwise
            disp('Environment not recognized!')
    end
    
    filename = 'init.mat';
    save(filename)
end