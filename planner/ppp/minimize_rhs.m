% Given a vertex, iterate through successors to get determine the minimum
% rhs value
function minimize_rhs(vertex, obs_set)
    min_rhs = inf;
    for(i=1:vertex.num_succ)
       succ_v = vertex.succ_list(i);
       if(succ_v.g ~= inf)
          cost = cost_to_vertex(vertex, succ_v, obs_set);
          cost = cost + succ_v.g;
           if(cost < min_rhs)
                min_rhs = cost;

                % Copy the state conditions required for this low cost rhs
                vertex.rhs = min_rhs;
           end 
       end
    end
end