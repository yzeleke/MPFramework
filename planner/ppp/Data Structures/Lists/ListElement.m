classdef ListElement < handle
%
% A class that implements pass by reference numeric elements for instances
% of the List class
%
% NOTE: This class is used internally by the List class
%

    %
    % Public properties
    %
	properties (Access = public)
		value;          % value
        prev = nan;     % previous element
        next = nan;     % next element
    end
    
    %
    % Public methods
    %
	methods (Access = public)
        %
        % Constructor
        %
		function this = ListElement(value)
            if (nargin == 1)
                % Initialize value
                this.value = value;
            end
        end
        
        %
        % Element is nan if its value is nan
        %
        function bool = isnan(this)
            bool = isnan(this.value);
        end
        
    end
end
