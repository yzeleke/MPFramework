classdef ppp_PQ2 < handle

    properties
        nElements
        valueList;
    end

    methods
        function self = ppp_PQ2()
            self.nElements = 0;
            self.valueList = [];
        end

        function push(self, vertex, priority)
            vertex.priority = priority;
            self.valueList = [self.valueList, vertex];
            self.nElements = self.nElements + 1;
        end


        function minPriorityElement = pop(self)
            if ~self.isEmpty
                
                min_v = self.valueList(1); % Min vertex
                min_i = 1;                 % Min index
                for(i=1:self.nElements)
                    if(self.valueList(i).priority < min_v.priority)
                        min_v = self.valueList(i);
                        min_i = i;
                    end
                end
                
                self.valueList(min_i) = [];
                self.nElements = self.nElements - 1;
                minPriorityElement = min_v;
            else
                disp('Queue is empty');
            end
        end
        
        function minKey = min_key(self)
           if(self.isEmpty())
                minKey = Inf;
           else
               min_v = self.valueList(1);
               for(i=1:self.nElements)
                  if(self.valueList(i).priority < min_v.priority)
                    min_v = self.valueList(i);
                  end
               end
               minKey = min_v.priority;
           end
        end
        
        function result = contains(self, vertex)
            % ismember returns an array of boolean values
            % where the index is the index of the match
            ans = ismember(self.valueList, vertex);
            result = sum(ans) ~= 0;
        end
        
        function remove(self, vertex)
            for(i=1:self.nElements)
                if(self.valueList(i) == vertex)
                    self.valueList(i) = [];
                    break;
                end
            end
            self.nElements = self.nElements - 1;
        end

        function flagIsEmpty = isEmpty(self)
            flagIsEmpty = (self.nElements == 0);
        end
    end
end
