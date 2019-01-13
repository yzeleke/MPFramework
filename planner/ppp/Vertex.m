classdef Vertex < matlab.mixin.Copyable
   properties
      g
      rhs
      x
      xv
      xa
      y
      yv
      ya
      x_traj
      y_traj
      next
      path_time
      time_res
      num_pred
      pred_list
      num_succ
      succ_list
      priority
   end
   methods
        function self = Vertex()
            self.path_time = 1;
            self.time_res  = 0.001;
            
        end
   end
end