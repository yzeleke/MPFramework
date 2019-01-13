classdef Array < handle
%--------------------------------------------------------------------------
% Class:        Array < handle
%               
% Constructor:  x = Array(x);
%               
% Inputs:       x = arbitrary MATLAB array
%               
% Properties:   (none)
%               
% Methods:      num  = numel(x);
%               len  = length(x);
%               siz  = size(x);
%               ndim = ndims(x);
%                      flipud(x);
%                      fliplr(x);
%                      flipdim(x,dim);
%                      display(x);
%               
% Destructor:   x = Return(x);
%                  
% Indexing:     All standard array indexing operations (EXCEPT the 'end'
%               keyword) are suppored. Examples include:
%               
%               x(i,j)    = 1;
%               x(2:2:10) = randn(5,1);
%               x(:)      = [];
%               xij       = x(i,j);
%               xodd      = x(1:2:9);
%               x
%               x(:)
%               
% Description:  This class converts the input standard MATLAB (i.e., pass
%               by value) array x to a pass by reference object
%               
% Author:       Brian Moore
%               brimoor@umich.edu
%               
% Date:         January 16, 2014
%--------------------------------------------------------------------------
    
    %
    % private properties
    %
    properties (Access = private)
        x;
    end
    
    %
    % Public methods
    %
    methods
        %
        % Constructor
        %
        function this = Array(x)
            %----------------------- Constructor --------------------------
            % Syntax:       x = Array(x);
            %               
            % Description:  Converts the input array x to a
            %               pass by reference object
            %--------------------------------------------------------------
            
            this.x = x;
        end
        
        %
        % Return value array
        %
        function x = Return(this)
            %------------------------ Return ------------------------------
            % Syntax:       x = Return(x);
            %               x = x.Return();
            %               
            % Description:  Converts the Array object to a standard MATLAB
            %               (i.e., pass by value) array
            %--------------------------------------------------------------
            
            x = this.x;
        end
        
        %
        % Define numel()
        %
        function num = numel(this)
            %------------------------- numel ------------------------------
            % Syntax:       num = numel(x);
            %               
            % Description:  Returns the number of elements in x
            %--------------------------------------------------------------
            
            num = numel(this.x);
        end
        
        %
        % Define length()
        %
        function len = length(this)
            %------------------------ length ------------------------------
            % Syntax:       len = length(x);
            %               
            % Description:  Returns the length of x
            %--------------------------------------------------------------
            
            len = length(this.x);
        end
        
        %
        % Define size()
        %
        function siz = size(this)
            %------------------------- size -------------------------------
            % Syntax:       siz = size(x);
            %               
            % Description:  Returns the dimensions of x
            %--------------------------------------------------------------
            
            siz = size(this.x);
        end
        
        %
        % Define ndims()
        %
        function ndim = ndims(this)
            %------------------------- ndims ------------------------------
            % Syntax:       ndim = ndims(x);
            %               
            % Description:  Returns the number of dimenions of x
            %--------------------------------------------------------------
            
            ndim = ndims(this.x);
        end
        
        %
        % Define flipud()
        %
        function flipud(this)
            %------------------------ flipud ------------------------------
            % Syntax:       flipud(x);
            %               
            % Description:  Flips x w.r.t. its first dimension
            %--------------------------------------------------------------
            
            this.x = flipud(this.x);
        end
        
        %
        % Define fliplr()
        %
        function fliplr(this)
            %------------------------ fliplr ------------------------------
            % Syntax:       fliplr(x);
            %               
            % Description:  Flips x w.r.t. its second dimension
            %--------------------------------------------------------------
            
            this.x = fliplr(this.x);
        end
        
        %
        % Define flipdim()
        %
        function flipdim(this,dim)
            %----------------------- flipdim ------------------------------
            % Syntax:       flipdim(x,dim);
            %               
            % Description:  Flips x w.r.t. dimension dim
            %--------------------------------------------------------------
            
            this.x = flipdim(this.x,dim);
        end
        
        %
        %
        % Define () for Array element assignment
        %
        function this = subsasgn(this,S,vals)
            %----------------- ()-based assignment ------------------------
            % Syntax (Ex):  x(1) = 1;
            %               x(1:2:10) = rand(5,1);
            %               x(:) = [];
            %               
            % Description:  Performs standard array assignment operations
            %               on x
            %               
            % Note:         'end' keyword is NOT supported...
            %--------------------------------------------------------------
             
            this.x = builtin('subsasgn',this.x,S,vals);
        end
        
        %
        % Define () for Array element referencing
        %
        function vals = subsref(this,S)
            %------------------ ()-based reference ------------------------
            % Syntax (Ex):  x1 = x(1);
            %               xodd = x(1:2:9);
            %               y = x(:);
            %               
            % Description:  Performs standard array reference operations
            %               on x
            %               
            % Note:         'end' keyword is NOT supported...
            %--------------------------------------------------------------
            
            vals = builtin('subsref',this.x,S);
        end
        
        %
        % Define display function
        %
        function display(this)
            %------------------ ()-based reference ------------------------
            % Syntax (Ex):  x            (no semicolon)
            %               x(1:2:9)     (no semicolon)
            %               display(x);
            %               
            % Description:  Returns the result of an indexing call to x
            %               
            % Note:         'end' keyword is NOT supported...
            %--------------------------------------------------------------
            
            this.x
        end
    end
end
