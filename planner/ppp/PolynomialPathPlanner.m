%%
%--------------------------------------------------------------------------
%							 PolynomialPathPlanner.m
%--------------------------------------------------------------------------
%**************************************************************************
% @ Author                   : 	David Kooi	
% @file_name				 :  PolynomialPathPlanner.m														  
% @ Date                     : 	11/02/18                                                     
% @ Description				 :    
%                                
%                               
% @ Usage					:   MpcPlanner(model) model is the
%                                   name of .mat file    																						  
% @Revision					:  	11/5/18                                                                                      
%***************************************************************************

function PolynomialPathPlanner(Tsim)
    % Load environment
    load('environment.mat');
    %load vehilce model
    load('model.mat');


