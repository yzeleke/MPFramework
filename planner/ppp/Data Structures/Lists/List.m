classdef List < handle
%--------------------------------------------------------------------------
% Class:        List < handle
%               
% Constructor:  L = List();
%               L = List(x0);
%               
% Properties:   (none)
%               
% Methods:               L.Insert(i,value);
%                        L.Prepend(value);
%                        L.Append(value);
%                        L.Set(i,value);
%               value  = L.Get(i);
%               values = L.GetAll();
%                        L.Sort();
%                        L.Delete(i);
%               bool   = L.Contains(value);
%               i      = L.Find(value);
%               count  = L.Count();
%               bool   = L.IsEmpty();
%                        L.Clear();
%               
% Indexing:     The following set/get indexing is supported:
%               
%               L(i)   = value;
%               value  = L(i);
%               
% Description:  This class implements a doubly linked list with numeric
%               values
%               
% Author:       Brian Moore
%               brimoor@umich.edu
%               
% Date:         January 16, 2014
%--------------------------------------------------------------------------

    %
    % Private constants
    %
    properties (GetAccess = private, Constant = true)
        kk = 15;                    % insertion/merge sort transition
    end
    
    %
    % Private properties
    %
    properties (Access = private)
        k;                          % current number of elements
        root;                       % pointer to list root    
        nil;                        % sentinel element
    end
    
    %
    % Public methods
    %
    methods (Access = public)
        %
        % Constructor
        %
        function this = List(x0)
            %----------------------- Constructor --------------------------
            % Syntax:       L = List();
            %               L = List(x0);
            %               
            % Inputs:       x0 is a vector of numeric values to insert into
            %               the list during initialization
            %               
            % Description:  Creates an empty (if x0 is not supplied) list
            %--------------------------------------------------------------
            
            % Start with an empty list
            this.Clear();
            
            % Insert elements, if any
            if (nargin == 1)
                % Insert values in the given order
                for i = 1:numel(x0)
                    this.Append(x0(i));
                end
            end
        end
        
        %
        % Insert value into list at given position
        %
        function Insert(this,i,value)
            %------------------------- Insert -----------------------------
            % Syntax:       L.Insert(i,value);
            %               
            % Inputs:       i is the desired index at which to insert the
            %               given value
            %               
            %               value is a number
            %               
            % Description:  Inserts the given value into L between existing
            %               indices i and i + 1
            %--------------------------------------------------------------
            
            % Append zeros if necessary
            while (i > (this.k + 1))
                this.Append(0);
            end
            
            % Create list element with given value
            x = ListElement(value);
            this.k = this.k + 1;
            
            % Insert element into list
            xp = this.IndexPointer(i - 1);
            if (this.k > 1)
                xn = xp.next;
            else
                xn = xp;
            end
            xp.next = x;
            x.prev = xp;
            x.next = xn;
            xn.prev = x;
        end
        
        %
        % Prepend value to list
        %
        function Prepend(this,value)
            %------------------------- Prepend ----------------------------
            % Syntax:       L.Prepend(value);
            %               
            % Inputs:       value is a number
            %               
            % Description:  Prepends the given value to L
            %--------------------------------------------------------------
            
            this.Insert(1,value);
        end
        
        %
        % Append value to list
        %
        function Append(this,value)
            %------------------------- Append -----------------------------
            % Syntax:       L.Append(value);
            %               
            % Inputs:       value is a number
            %               
            % Description:  Appends the given value to L
            %--------------------------------------------------------------
            
            this.Insert(this.k + 1,value);
        end
        
        %
        % Define () for List element assignment
        %
        % NOTE: MATLAB's deplorable implementation of subsasgn() makes this
        % method extremeley clunky...
        %
        function this = subsasgn(this,S,value)
            %------------------ ()-based assignment -----------------------
            % Syntax:       L(i) = value;
            %               
            % Inputs:       i is a list index
            %               
            %               value is a number
            %               
            % Description:  Sets the ith index of L to the given value
            %--------------------------------------------------------------
            
            switch S(1).type
                case '()'
                    % User is setting a list element
                    i = S(1).subs{1};
                    this.Set(i,value);
                case '{}'
                    % {}-indexing is not supported 
                    List.CellIndexingError();
                case '.'
                    if isprop(this,S(1).subs)
                        % User tried to set a property (assumed private)
                        List.PrivatePropertySetError(S(1).subs);
                    else
                        % Call built-in assignment method
                        this = builtin('subsasgn',this,S,value);
                    end
            end
        end
        
        %
        % Sets element at given position
        %
        function Set(this,i,value)
            %--------------------------- Set ------------------------------
            % Syntax:       L.Set(i,value);
            %               
            % Inputs:       value is a number
            %               
            % Description:  Sets the ith index of L to the given value
            %--------------------------------------------------------------
            
            x = this.IndexPointer(i);
            x.value = value;
        end
        
        %
        % Define () for List element referencing
        %
        % NOTE: MATLAB's deplorable implementation of subsref() makes this
        % method extremeley clunky...
        %
        function varargout = subsref(this,S)
            %------------------- ()-based reference -----------------------
            % Syntax:       value = L(i);
            %               
            % Inputs:       i is a list index
            %               
            % Outputs:      value is a number
            %               
            % Description:  Sets the ith index of L to the given value
            %--------------------------------------------------------------
            
            switch S(1).type
                case '()'
                    % User is getting a value from the list
                    i = S(1).subs{1};
                    varargout{1} = this.Get(i);
                case '{}'
                    % {}-indexing is not supported
                    List.CellIndexingError();
                case '.'
                    % Call built-in reference method
                    if (isprop(this,S(1).subs) || (nargout(['List>List.' S(1).subs]) > 0))
                        % Doesn't assign output args >= 2, if they exist
                        varargout{1} = builtin('subsref',this,S);
                    else
                        builtin('subsref',this,S);
                    end
            end
        end
        
        %
        % Gets value at given position
        %
        function value = Get(this,i)
            %--------------------------- Get ------------------------------
            % Syntax:       value = L.Get(i);
            %               
            % Inputs:       i is a list index
            %               
            % Outputs:      value is a number
            %               
            % Description:  Returns the value in the ith index of L
            %--------------------------------------------------------------
            
            value = this.IndexPointer(i).value;
        end
        
        %
        % Get the entire list of values
        %
        function values = GetAll(this)
            %------------------------- GetAll -----------------------------
            % Syntax:       values = L.GetAll();
            %                             
            % Outputs:      values is an array containg the values in L
            %               
            % Description:  Returns the values in L
            %--------------------------------------------------------------
            
            values = nan(this.k,1);
            i = 1;
            x = this.root.next;
            while (i <= this.k)
                values(i) = x.value;
                i = i + 1;
                x = x.next;
            end
        end
        
        %
        % Sort the list via optimized merge sort
        %
        function Sort(this)
            %-------------------------- Sort ------------------------------
            % Syntax:       L.Sort();
            %                             
            % Description:  Sorts (ascending order) the values in L 
            %--------------------------------------------------------------
            
            if (this.k >= 2)
                % Merge sort (with inner switch to insertion sort)
                [first last] = List.MergeSorti(this.root.next,this.k);
                
                % Update root pointers
                this.root.next = first;
                first.prev = this.root;
                this.root.prev = last;
                last.next = this.root;
            end
        end
        
        %
        % Delete value at given position from list
        %
        function Delete(this,i)
            %------------------------- Delete -----------------------------
            % Syntax:       L.Delete(i);
            %               
            % Inputs:       i is a list index
            %               
            % Description:  Removes the ith value from L
            %--------------------------------------------------------------
            
            x = this.IndexPointer(i);
            this.k = this.k - 1;
            x.prev.next = x.next;
            x.next.prev = x.prev;
        end
        
        %
        % Search for an element with the given value
        %
        function bool = Contains(this,value)
            %------------------------ Contains ----------------------------
            % Syntax:       bool = L.Contains(value);
            %               
            % Inputs:       value is a number
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if L contains the given value
            %--------------------------------------------------------------
            
            bool = ~isempty(this.Find(value));
        end
        
        %
        % Return list index to (first) element in list with given value
        %
        function i = Find(this,value)
            %-------------------------- Find ------------------------------
            % Syntax:       i = L.Find(value);
            %               
            % Inputs:       value is a number
            %               
            % Outputs:      i is the index of the (first) occurance of the
            %               given value in L
            %               
            % Description:  Returns the index of the (first) occurance of
            %               the given value in L
            %--------------------------------------------------------------
            
            i = 1;
            x = this.root.next; % point to first element
            while (~isnan(x) && (x.value ~= value))
                x = x.next;
                i = i + 1;
            end
            if (i > this.k)
                i = [];
            end
        end
        
        %
        % Return number of elements in list
        %
        function count = Count(this)
            %-------------------------- Count -----------------------------
            % Syntax:       count = L.Count();
            %               
            % Outputs:      count is the number of values in L
            %               
            % Description:  Returns the number of values in L
            %--------------------------------------------------------------
            
            count = this.k;
        end
        
        %
        % Check for empty list
        %
        function bool = IsEmpty(this)
            %------------------------- IsEmpty ----------------------------
            % Syntax:       bool = L.IsEmpty();
            %               
            % Outputs:      bool = {true,false}
            %               
            % Description:  Determines if L is empty (i.e., contains zero
            %               values)
            %--------------------------------------------------------------
            
            if (this.k == 0)
                bool = true;
            else
                bool = false;
            end
        end
        
        %
        % Clear the list
        %
        function Clear(this)
            %-------------------------- Clear -----------------------------
            % Syntax:       L.Clear();
            %               
            % Description:  Removes all value from L
            %--------------------------------------------------------------
            
            this.k = 0;                     % reset length counter
            this.nil = ListElement(nan);    % reset nil pointer
            this.root = this.nil;           % reset root pointer
        end
    end
    
    %
    % Private methods
    %
    methods (Access = private)
        %
        % Return element at list index i
        %
        function x = IndexPointer(this,i)
            if (i > this.k)
                List.IndexOverflowError();
            end
            x = this.root; % point to sentinel
            for iter = 1:i
                x = x.next;
            end
        end
    end
    
    %
    % Private static methods
    %
    methods (Access = private, Static = true)
        %
        % Inner insertion sort routine
        %
        function ll = InsertionSorti(ll,count)
            x = ll.next; % second element
            j = 2;
            while (j <= count)
                pivot = x.value;
                y = x;
                i = j;
                while ((i > 1) && (y.prev.value > pivot))
                    y.value = y.prev.value;
                    y = y.prev;
                    i = i - 1;
                end
                y.value = pivot;
                x = x.next;
                j = j + 1;
            end
        end
        
        %
        % Inner merge sort routine 
        %
        function [head foot] = MergeSorti(ll,count)
            if (count <= 1)
                head = ll;
                foot = ll;
            else
                % Find middle element
                mm = ll;
                i = 1;
                countL = floor(count / 2);
                countR = count - countL;
                while (i <= countL)
                    mm = mm.next;
                    i = i + 1;
                end
                
                % Divide-and-conquer with optimized switch between merge
                % and inerstion sort
                if (countL <= List.kk)
                    headL = List.InsertionSorti(ll,countL);
                else
                    [headL ~] = List.MergeSorti(ll,countL);
                end
                if (countR <= List.kk)
                    headR = List.InsertionSorti(mm,countR);
                else
                    [headR ~] = List.MergeSorti(mm,countR);
                end
                
                % Merge results
                [head foot] = List.Merge(headL,headR,countL,countR);
            end
        end
        
        %
        % Merge routine
        %
        function [head foot] = Merge(headL,headR,countL,countR)
            head = ListElement();
            foot = head;
            while ((countL > 0) || (countR > 0))
                if (headL.value <= headR.value)
                    if (countL > 0)
                        foot.next = headL;
                        headL = headL.next;
                        countL = countL - 1;
                    else
                        foot.next = headR;
                        headR = headR.next;
                        countR = countR - 1;
                    end
                else
                    if (countR > 0)
                        foot.next = headR;
                        headR = headR.next;
                        countR = countR - 1;
                    else
                        foot.next = headL;
                        headL = headL.next;
                        countL = countL - 1;
                    end
                end
                foot.next.prev = foot;
                foot = foot.next;
            end
            head = head.next;
        end
        
        %
        % Index overflow error
        %
        function IndexOverflowError()
            error('List index overflow');
        end
        
        %
        % Cell indexing error
        %
        function CellIndexingError()
            error('{} not supported for List indexing. Use () instead');
        end
        
        %
        % Private property set error
        %
        % Note: This is similar to MATLAB's standard error message when
        % one tries to set a private set property of an object
        %
        function PrivatePropertySetError(field)
            error('Setting the ''%s'' property of the ''List'' class is not allowed',field);
        end
    end
end
